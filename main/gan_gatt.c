// cubebridge - ESP32 BLE gateway that bridges physical smart cubes
// into virtual GAN cubes for GAN-protocol apps.
// Copyright (C) 2026 The cubebridge authors
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, version 3 of the License.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// GAN Gen2 peripheral. BLE state/crypto/TX belong exclusively to the host task.
// g_vcube, battery and command responses belong exclusively to the main task.
#include "gan_gatt.h"
#include "gan_crypto.h"
#include "gan_proto.h"
#include "gan_address.h"
#include "cube_client.h"
#include "cube_state.h"
#include <string.h>
#include <stdatomic.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nimble/nimble_npl.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

extern vcube_t g_vcube;
static const char *TAG = "gan";
static uint8_t s_mac_le[6];
// 128 位 UUID 用 BLE_UUID128_INIT 按小端字节序声明
// service 6E400001-B5A3-F393-E0A9-E50E24DC4179
static const ble_uuid128_t svc_uuid = BLE_UUID128_INIT(
    0x79, 0x41, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);
// notify 28BE4CB6-CD67-11E9-A32F-2A2AE2DBCCE4
static const ble_uuid128_t notify_uuid = BLE_UUID128_INIT(
    0xE4, 0xCC, 0xDB, 0xE2, 0x2A, 0x2A, 0x2F, 0xA3,
    0xE9, 0x11, 0x67, 0xCD, 0xB6, 0x4C, 0xBE, 0x28);
// write 28BE4A4A-CD67-11E9-A32F-2A2AE2DBCCE4
static const ble_uuid128_t write_uuid = BLE_UUID128_INIT(
    0xE4, 0xCC, 0xDB, 0xE2, 0x2A, 0x2A, 0x2F, 0xA3,
    0xE9, 0x11, 0x67, 0xCD, 0x4A, 0x4A, 0xBE, 0x28);


static uint8_t s_own_addr_type = BLE_OWN_ADDR_RANDOM;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_notify_handle;
static gan_crypto_t s_crypto;
static bool s_host_subscribed;
static struct ble_npl_callout s_tx_tick;
static bool s_tick_initialized;
static void (*s_ready_cb)(void);

// Cross-task state is atomic. Epochs also distinguish reused connection handles.
static atomic_uint s_session = 1;
static atomic_uint s_source_generation;
static atomic_bool s_subscribed;
static atomic_uint s_snapshot_session;
static atomic_bool s_close_pending;
static atomic_bool s_fault;
static uint8_t s_battery; // main task only; 0 until source reports a value
enum { CMD_INITIAL = 100 };
typedef struct { uint32_t session; int cmd; } command_t;
typedef struct { uint32_t session; uint8_t frame[GAN_FRAME_LEN]; } tx_t;
static QueueHandle_t s_commands, s_tx;
static bool s_tx_pending;
static tx_t s_pending;
static int64_t s_pending_since;

static bool bridge_available(void) {
    return !atomic_load(&s_fault) &&
        cube_client_generation_valid(atomic_load(&s_source_generation));
}

void gan_gatt_set_bridge_ready(uint32_t generation) {
    if (!generation) {
        if (atomic_exchange(&s_source_generation, 0))
            atomic_store(&s_close_pending, true);
        atomic_store(&s_subscribed, false);
    } else if (!atomic_load(&s_fault) && cube_client_generation_valid(generation)) {
        atomic_store(&s_source_generation, generation);
    }
}

static void transport_fault(const char *reason) {
    ESP_LOGW(TAG, "Bridge session invalid: %s", reason);
    atomic_store(&s_fault, true);
    gan_gatt_set_bridge_ready(0);
    atomic_store(&s_close_pending, true);
}

bool gan_gatt_take_fault(void) { return atomic_exchange(&s_fault, false); }

static void log_hex(const char *tag, const uint8_t *data, int len) {
    char hex[61] = {0};
    for (int i = 0; i < len && i < 20; i++) snprintf(hex + i * 3, 4, "%02X ", data[i]);
    ESP_LOGD(TAG, "%s %s", tag, hex);
}

static const char *access_op_name(uint8_t op) {
    return op == BLE_GATT_ACCESS_OP_READ_CHR ? "READ" : "WRITE";
}

static bool post_command(int cmd) {
    command_t request = { atomic_load(&s_session), cmd };
    if (xQueueSend(s_commands, &request, 0) != pdTRUE) {
        transport_fault("command queue overflow");
        return false;
    }
    return true;
}

static bool enqueue_frame(const uint8_t *frame, uint32_t session) {
    if (!bridge_available() || !atomic_load(&s_subscribed) ||
        session != atomic_load(&s_session)) return false;
    tx_t item = { .session = session };
    memcpy(item.frame, frame, GAN_FRAME_LEN);
    if (xQueueSend(s_tx, &item, 0) != pdTRUE) {
        transport_fault("TX queue overflow");
        return false;
    }
    return true;
}

bool gan_gatt_notify(const uint8_t *frame, size_t len) {
    uint32_t session = atomic_load(&s_session);
    if (!frame || len != GAN_FRAME_LEN || atomic_load(&s_snapshot_session) != session) return false;
    return enqueue_frame(frame, session);
}

static int notify_access_cb(uint16_t conn, uint16_t attr,
                           struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_READ_NOT_PERMITTED;
    // Existing compatibility behavior, pending a real-device profile capture.
    const uint8_t zero[GAN_FRAME_LEN] = {0};
    return os_mbuf_append(ctxt->om, zero, sizeof(zero)) == 0
        ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int write_access_cb(uint16_t conn, uint16_t attr,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    if (OS_MBUF_PKTLEN(ctxt->om) != GAN_FRAME_LEN) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    if (conn != s_conn_handle || !bridge_available()) return BLE_ATT_ERR_UNLIKELY;
    uint8_t req[GAN_FRAME_LEN];
    uint16_t copied;
    if (ble_hs_mbuf_to_flat(ctxt->om, req, sizeof(req), &copied) != 0 || copied != sizeof(req))
        return BLE_ATT_ERR_UNLIKELY;
    gan_crypto_decode(&s_crypto, req, sizeof(req));
    int cmd = gan_proto_dispatch(req, sizeof(req));
    if (cmd == PROTO_CMD_UNKNOWN) return 0;
    return post_command(cmd) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// GATT 服务定义
// ---------------------------------------------------------------------------
static const ble_uuid16_t dis_uuid = BLE_UUID16_INIT(GAN_DIS_UUID);
static const ble_uuid16_t dis_model_uuid = BLE_UUID16_INIT(GAN_DIS_MODEL_UUID);
static const ble_uuid16_t dis_firmware_uuid = BLE_UUID16_INIT(GAN_DIS_FIRMWARE_UUID);
static const ble_uuid16_t dis_hardware_uuid = BLE_UUID16_INIT(GAN_DIS_HARDWARE_UUID);
static const ble_uuid16_t dis_vendor_uuid = BLE_UUID16_INIT(GAN_DIS_VENDOR_UUID);

static int device_info_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg) {
    uint16_t uuid = ble_uuid_u16(ctxt->chr->uuid);
    ESP_LOGI(TAG, "[gan-rx] GATT操作 conn=%u attr=%u op=%s(%d) 设备信息 uuid=0x%04X",
             conn_handle, attr_handle, access_op_name(ctxt->op), ctxt->op, uuid);
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_READ_NOT_PERMITTED;
    const char *value = gan_identity_value(uuid);
    if (value == NULL) return BLE_ATT_ERR_UNLIKELY;
    ESP_LOGI(TAG, "[gan-info] 读取设备信息 conn=%u uuid=0x%04X value=%s",
             conn_handle, uuid, value);
    // 标准 DIS 为 UTF-8 文本，不包含结尾 NUL，不使用 GAN 应用层加密。
    return os_mbuf_append(ctxt->om, value, strlen(value)) == 0
        ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (const ble_uuid_t *)&svc_uuid,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = (const ble_uuid_t *)&notify_uuid,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .access_cb = notify_access_cb,
                .val_handle = &s_notify_handle,
            },
            {
                .uuid = (const ble_uuid_t *)&write_uuid,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .access_cb = write_access_cb,
            },
            { 0 },
        },
    },
    // 追加在 GAN 服务之后，保留已有 GAN 特征的注册顺序。
    // 兼容直接读取标准型号/版本的客户端；不假定魔方星球一定使用 DIS。
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (const ble_uuid_t *)&dis_uuid,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = (const ble_uuid_t *)&dis_model_uuid,
                .flags = BLE_GATT_CHR_F_READ,
                .access_cb = device_info_access_cb,
            },
            {
                .uuid = (const ble_uuid_t *)&dis_firmware_uuid,
                .flags = BLE_GATT_CHR_F_READ,
                .access_cb = device_info_access_cb,
            },
            {
                .uuid = (const ble_uuid_t *)&dis_hardware_uuid,
                .flags = BLE_GATT_CHR_F_READ,
                .access_cb = device_info_access_cb,
            },
            {
                .uuid = (const ble_uuid_t *)&dis_vendor_uuid,
                .flags = BLE_GATT_CHR_F_READ,
                .access_cb = device_info_access_cb,
            },
            { 0 },
        },
    },
    { 0 },
};

static int gap_event_handler(struct ble_gap_event *event, void *arg);

static void start_advertising(void) {
    if (!bridge_available() || atomic_load(&s_close_pending) ||
        s_conn_handle != BLE_HS_CONN_HANDLE_NONE || ble_gap_adv_active()) return;
    // 厂商数据: CIC(2字节, 0x0101) + 占位3字节 + 显示地址逆序6字节
    uint8_t mf[11] = {
        0x01, 0x01,
        0x00, 0x00, 0x00,
    };
    memcpy(mf + 5, s_mac_le, sizeof(s_mac_le));
    log_hex("[gan-adv] 厂商数据:", mf, sizeof(mf));

    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.mfg_data = mf;
    fields.mfg_data_len = sizeof(mf);
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv set fields failed: %d", rc);
        return;
    }

    // 名称放入扫描响应
    struct ble_hs_adv_fields sr;
    memset(&sr, 0, sizeof(sr));
    sr.name = (uint8_t *)GAN_DEVICE_NAME;
    sr.name_len = strlen(GAN_DEVICE_NAME);
    sr.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&sr);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv rsp set fields failed: %d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv start failed: %d", rc);
    }
}


// Host-only reset: queued work from earlier epochs cannot reach a new client.
static void new_session(void) {
    atomic_store(&s_subscribed, false);
    atomic_store(&s_snapshot_session, 0);
    s_host_subscribed = false;
    atomic_fetch_add(&s_session, 1);
    s_tx_pending = false;
}

static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                new_session();
                s_conn_handle = event->connect.conn_handle;
                if (!bridge_available() || atomic_load(&s_close_pending))
                    atomic_store(&s_close_pending, true);
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            if (event->disconnect.conn.conn_handle == s_conn_handle) {
                s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
                new_session();
            }
            break;
        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.conn_handle != s_conn_handle ||
                event->subscribe.attr_handle != s_notify_handle) break;
            if (!event->subscribe.cur_notify) {
                new_session();
            } else if (!s_host_subscribed && bridge_available() &&
                       !atomic_load(&s_close_pending)) {
                s_host_subscribed = true;
                // Queue initial snapshot before making MOVE enqueue possible.
                if (post_command(CMD_INITIAL)) atomic_store(&s_subscribed, true);
            }
            break;
        case BLE_GAP_EVENT_NOTIFY_TX:
            if (event->notify_tx.conn_handle == s_conn_handle &&
                event->notify_tx.attr_handle == s_notify_handle && event->notify_tx.status != 0)
                transport_fault("asynchronous notification failure");
            break;
        default: break;
    }
    return 0;
}

static void tx_tick(struct ble_npl_event *event) {
    if (!bridge_available() || atomic_load(&s_close_pending)) {
        atomic_store(&s_subscribed, false);
        s_host_subscribed = false;
        s_tx_pending = false;
        if (ble_gap_adv_active()) ble_gap_adv_stop();
        if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            int rc = ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            if (rc && rc != BLE_HS_EALREADY) ESP_LOGW(TAG, "terminate rc=%d", rc);
            // Keep closure latched until the disconnect callback.
        } else {
            atomic_store(&s_close_pending, false);
        }
        tx_t discard;
        while (xQueueReceive(s_tx, &discard, 0) == pdTRUE) {}
        goto rearm;
    }
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) start_advertising();
    if (!s_host_subscribed) goto rearm;
    for (int count = 0; count < 8; count++) {
        if (!s_tx_pending) {
            if (xQueueReceive(s_tx, &s_pending, 0) != pdTRUE) break;
            s_pending_since = esp_timer_get_time();
            s_tx_pending = true;
        }
        if (s_pending.session != atomic_load(&s_session)) {
            s_tx_pending = false;
            continue;
        }
        if (!bridge_available() || atomic_load(&s_close_pending)) break;
        uint8_t encrypted[GAN_FRAME_LEN];
        memcpy(encrypted, s_pending.frame, sizeof(encrypted));
        gan_crypto_encode(&s_crypto, encrypted, sizeof(encrypted));
        struct os_mbuf *om = ble_hs_mbuf_from_flat(encrypted, sizeof(encrypted));
        int rc = om ? ble_gatts_notify_custom(s_conn_handle, s_notify_handle, om) : BLE_HS_ENOMEM;
        if (rc == 0) { s_tx_pending = false; continue; }
        // Retry the same frame in order. Never skip a failed state response or
        // rely on a later move arriving to repair an unknown notification loss.
        if ((rc != BLE_HS_ENOMEM && rc != BLE_HS_EBUSY && rc != BLE_HS_EAGAIN) ||
            esp_timer_get_time() - s_pending_since >= 250000) {
            transport_fault("notification could not be submitted within 250ms");
        }
        break;
    }
rearm:
    ble_npl_callout_reset(&s_tx_tick, pdMS_TO_TICKS(10));
}

void gan_gatt_service(void) {
    // Main-task only. Commands before CCCD remain queued until subscription.
    command_t request;
    for (int i = 0; i < 16 && xQueuePeek(s_commands, &request, 0) == pdTRUE; i++) {
        if (request.session != atomic_load(&s_session) || !bridge_available()) {
            xQueueReceive(s_commands, &request, 0);
            continue;
        }
        if (request.cmd != PROTO_CMD_RESET && !atomic_load(&s_subscribed)) break;
        xQueueReceive(s_commands, &request, 0);
        uint8_t frame[GAN_FRAME_LEN];
        switch (request.cmd) {
            case PROTO_CMD_RESET:
                // No verified upstream calibration command exists. End the
                // session and resynchronize the real cube; never fake solved.
                transport_fault("RESET requires upstream resynchronization");
                return;
            case CMD_INITIAL:
                gan_proto_hardware(frame);
                if (!enqueue_frame(frame, request.session)) return;
                gan_proto_facelets(frame, g_vcube.serial, &g_vcube.cubie);
                if (!enqueue_frame(frame, request.session)) return;
                gan_proto_battery(frame, s_battery);
                if (!enqueue_frame(frame, request.session)) return;
                // Publishing the epoch (rather than a boolean) remains safe
                // even if the host disconnects between comparison and store.
                atomic_store(&s_snapshot_session, request.session);
                break;
            case PROTO_CMD_FACELETS:
                gan_proto_facelets(frame, g_vcube.serial, &g_vcube.cubie);
                if (!enqueue_frame(frame, request.session)) return;
                break;
            case PROTO_CMD_HARDWARE:
                gan_proto_hardware(frame);
                if (!enqueue_frame(frame, request.session)) return;
                break;
            case PROTO_CMD_BATTERY:
                gan_proto_battery(frame, s_battery);
                if (!enqueue_frame(frame, request.session)) return;
                break;
        }
    }
}

void gan_gatt_set_battery(uint8_t percent) { s_battery = percent > 100 ? 100 : percent; }
uint8_t gan_gatt_get_battery(void) { return s_battery; }
bool gan_gatt_is_subscribed(void) { return bridge_available() && atomic_load(&s_subscribed); }
void gan_gatt_set_ready_callback(void (*cb)(void)) { s_ready_cb = cb; }

static esp_err_t load_identity(void) {
    // A copied NVS image must not clone another board's BLE identity.
    uint8_t owner[6], record[13]; // version, owner eFuse MAC, random-static address
    esp_err_t rc = esp_efuse_mac_get_default(owner);
    if (rc != ESP_OK) return rc;
    nvs_handle_t handle;
    rc = nvs_open("ganidentity", NVS_READWRITE, &handle);
    if (rc != ESP_OK) return rc;
    size_t len = sizeof(record);
    rc = nvs_get_blob(handle, "address", record, &len);
    if (rc == ESP_OK && (len != sizeof(record) || record[0] != 1 || !gan_address_valid(record + 7))) {
        rc = ESP_ERR_INVALID_STATE; // Fail visibly; do not rotate corrupted identities.
    } else if (rc == ESP_ERR_NVS_NOT_FOUND ||
               (rc == ESP_OK && memcmp(record + 1, owner, 6) != 0)) {
        record[0] = 1;
        memcpy(record + 1, owner, 6);
        do {
            esp_fill_random(record + 7, 6);
            for (int i = 0; i < 6; i++) record[7 + i] ^= owner[i];
            record[12] |= 0xc0;
        } while (!gan_address_valid(record + 7));
        rc = nvs_set_blob(handle, "address", record, sizeof(record));
        if (rc == ESP_OK) rc = nvs_commit(handle);
    }
    if (rc == ESP_OK) memcpy(s_mac_le, record + 7, 6);
    nvs_close(handle);
    return rc;
}

static void on_sync(void) {
    ESP_ERROR_CHECK(ble_hs_id_set_rnd(s_mac_le));
    ESP_LOGI(TAG, "Bridge address: %02X:%02X:%02X:%02X:%02X:%02X",
        s_mac_le[5], s_mac_le[4], s_mac_le[3], s_mac_le[2], s_mac_le[1], s_mac_le[0]);
    if (!s_tick_initialized) {
        ble_npl_callout_init(&s_tx_tick, nimble_port_get_dflt_eventq(), tx_tick, NULL);
        s_tick_initialized = true;
    }
    ble_npl_callout_reset(&s_tx_tick, pdMS_TO_TICKS(10));
    if (s_ready_cb) s_ready_cb(); // Start source discovery; no GAN advertising yet.
}

static void on_reset(int reason) {
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    new_session();
    if (s_tick_initialized) ble_npl_callout_stop(&s_tx_tick);
    transport_fault("NimBLE host reset");
}

static void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void gan_gatt_init(void) {
    // Never automatically erase NVS: it contains the persistent BLE identity.
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(load_identity());
    uint8_t display[6];
    for (int i = 0; i < 6; i++) display[i] = s_mac_le[5 - i];
    gan_crypto_init(&s_crypto, 0, display);
    s_commands = xQueueCreate(16, sizeof(command_t));
    s_tx = xQueueCreate(32, sizeof(tx_t));
    configASSERT(s_commands && s_tx);
    ESP_ERROR_CHECK(nimble_port_init());
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_ERROR_CHECK(ble_gatts_count_cfg(gatt_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(gatt_svcs));
    ESP_ERROR_CHECK(ble_svc_gap_device_name_set(GAN_DEVICE_NAME));
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    nimble_port_freertos_init(ble_host_task);
}
