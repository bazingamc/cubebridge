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

// cube_client.c - BLE Central 实体魔方客户端 (ESP-IDF NimBLE)
//
// 功能: 扫描/连接/订阅 魔域(MHC / WCU_MY3) / 奇艺(QY-QYSC) 智能魔方,
//       解析转动与状态事件投递给主循环, 由 vcube 转成 GAN 协议转发魔方星球。
//
// 线程模型: 全部 BLE 操作在 NimBLE host 任务上下文执行
//   - GAP/GATT 回调 (扫描事件/连接/GATT发现/通知)
//   - 250ms 周期 callout (重试/超时/重连状态机)
//   主循环只经 FreeRTOS 队列消费事件 (cube_client_poll), 无跨任务 BLE 调用。
//
// 协议参考 (逆向来源):
//   魔域 MHC : csTimer moyucube.js / 基于作者私有 Flutter 项目移植
//              服务 0x1000, 明文; 0x1001写 0x1002读(分片) 0x1003转动
//   魔域 MY3 : csTimer moyu32cube.js / 基于作者私有 Flutter 项目移植
//              服务 0783b03e-...; GAN 同款 AES-CBC (MAC 派生密钥);
//              0xA1信息 0xA3面贴 0xA4电量 0xA5转动 0xAB陀螺仪 0xAC陀螺仪开关
//   奇艺     : csTimer qiyicube.js / 基于作者私有 Flutter 项目移植
//              服务 0xFFF0 特征 0xFFF6; AES-128-ECB 固定密钥 + CRC16-Modbus;
//              opcode 0x02 hello 0x03 状态变化 0x04 同步确认
#include "cube_client.h"
#include "cube_state.h"
#include "cube_time.h"
#include <stdatomic.h>
#include "gan_crypto.h"
#include "gan_gatt.h"

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs_adv.h"
#include "nimble/nimble_npl.h"
#include "aes/esp_aes.h"

#define TAG "cubeclient"

// ===========================================================================
// 协议常量
// ===========================================================================

// ---- 魔域 MHC (服务 00001000-0000-1000-8000-00805F9B34FB) ----
#define MHC_SVC_UUID    0x1000
#define MHC_CHR_WRITE   0x1001
#define MHC_CHR_READ    0x1002   // 分片响应 (通知)
#define MHC_CHR_TURN    0x1003   // 转动事件 (通知)
static const ble_uuid16_t MHC_SVC_UUID16 = BLE_UUID16_INIT(MHC_SVC_UUID);
// MHC 设备面序 -> URFDLB
static const uint8_t MHC_AXIS_MAP[6] = {3, 4, 5, 1, 2, 0};

// ---- 魔域 MY3/WCU_MY3 (服务 0783B03E-7735-B5A0-1760-A305D2795CB0) ----
static const ble_uuid128_t MY3_SVC_UUID = BLE_UUID128_INIT(
    0xB0, 0x5C, 0x79, 0xD2, 0x05, 0xA3, 0x60, 0x17,
    0xA0, 0xB5, 0x35, 0x77, 0x3E, 0xB0, 0x83, 0x07);
static const ble_uuid128_t MY3_CHR_READ_UUID = BLE_UUID128_INIT(
    0xB1, 0x5C, 0x79, 0xD2, 0x05, 0xA3, 0x60, 0x17,
    0xA0, 0xB5, 0x35, 0x77, 0x3E, 0xB0, 0x83, 0x07);
static const ble_uuid128_t MY3_CHR_WRITE_UUID = BLE_UUID128_INIT(
    0xB2, 0x5C, 0x79, 0xD2, 0x05, 0xA3, 0x60, 0x17,
    0xA0, 0xB5, 0x35, 0x77, 0x3E, 0xB0, 0x83, 0x07);
// MY3 根密钥/IV (与 GAN 同派生方案: key[i] = (base[i] + mac[5-i]) % 255, i<6)
static const uint8_t MY3_KEY_BASE[16] = {
    21, 119, 58, 92, 103, 14, 45, 31, 23, 103, 42, 19, 155, 103, 82, 87};
static const uint8_t MY3_IV_BASE[16] = {
    17, 35, 38, 37, 134, 42, 44, 59, 85, 6, 127, 49, 126, 103, 33, 87};

// ---- 奇艺 (服务 0000FFF0, 特征 0000FFF6) ----
#define QY_SVC_UUID 0xFFF0
#define QY_CHR_UUID 0xFFF6
static const ble_uuid16_t QY_SVC_UUID16 = BLE_UUID16_INIT(QY_SVC_UUID);
static const uint8_t QY_AES_KEY[16] = {
    87, 177, 249, 171, 205, 90, 232, 167, 156, 185, 140, 231, 87, 140, 81, 8};
// 奇艺转动编码 -> URFDLB (raw 1-12: axis = AXIS_MAP[(raw-1)>>1], 奇数=CCW)
static const uint8_t QY_AXIS_MAP[6] = {4, 1, 3, 0, 2, 5};

#define QY_MAX_FRAME 96           // 奇艺通知帧上限 (16 倍数)

// ===========================================================================
// 客户端状态机 (全部运行于 NimBLE host 任务)
// ===========================================================================
typedef enum {
    PH_IDLE = 0,
    PH_SCAN,          // 扫描中
    PH_CONNECT_WAIT,  // ble_gap_connect 等待中
    PH_MTU,           // MTU 协商中
    PH_DISC_SVC,      // 服务发现中
    PH_DISC_CHR,      // 特征发现中
    PH_DISC_DSC,      // 描述符发现中 (逐特征)
    PH_SUB_WRITE,     // CCCD 写入中 (逐特征)
    PH_SYNC,          // 已订阅, 等待初始状态
    PH_RUN,           // 正常转发
    PH_RECONN_WAIT,   // 掉线, 等待重连/重扫
} phase_t;

// 扫描候选设备
typedef struct {
    ble_addr_t addr;
    int8_t rssi;
    cube_dev_type_t type;      // NONE = 未识别
    uint8_t mac[6];            // 显示序 MAC (MY3/QY 密钥与握手用)
    bool has_mac;
    char name[24];
    uint8_t mfg[14];           // 原始厂商数据缓存 (名称与厂商数据可能分帧到达)
    uint8_t mfg_len;
    bool id_logged;            // 首次识别日志已打印
} cand_t;

// 特征记录 (发现阶段缓存)
typedef struct {
    uint16_t def_h;
    uint16_t val_h;
    ble_uuid_any_t uuid;
} chr_rec_t;

// 需订阅的通知特征
typedef struct {
    uint16_t val_h;
    uint16_t next_def;   // 下一特征 def_handle (或服务 end+1)
} notify_tgt_t;

static QueueHandle_t s_evtq;
static struct ble_npl_callout s_tick;
static bool s_tick_inited;

static _Atomic(cube_client_state_t) s_ext_state = CUBE_CLIENT_STOPPED;
static phase_t s_ph = PH_IDLE;
static uint32_t s_ph_ms;         // 当前阶段进入时间
static uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;

static cube_dev_type_t s_devtype = CUBE_DEV_NONE;
static uint8_t s_cubemac[6];     // 显示序 MAC
static char s_cubename[24];
static ble_addr_t s_last_addr;  // 最近一次连接成功的地址 (重连用)
static bool s_have_last_addr;

static cand_t s_cands[4];
static chr_rec_t s_chrs[12];
static int s_chr_n;
static uint16_t s_svc_start, s_svc_end;
static notify_tgt_t s_notify[3];
static int s_notify_n;
static uint8_t s_notify_log_n;   // 已打印十六进制日志的通知帧数 (每连接限 8 帧)
static uint16_t s_write_h;       // 写特征句柄
static int s_sub_i;
static uint16_t s_cccd_h;

static bool s_sync_done;
static uint8_t s_sync_retries;
static uint32_t s_last_req_ms;
static atomic_uchar s_cmd;  // 1=rescan 2=clear+rescan 3=resync
static atomic_uint s_generation = 1;
static atomic_bool s_stream_fault = true;

static void invalidate_stream(void) {
    if (!atomic_exchange(&s_stream_fault, true)) atomic_fetch_add(&s_generation, 1);
    gan_gatt_set_bridge_ready(0);
}

static void stream_fault(const char *why) {
    ESP_LOGW(TAG, "Invalid source stream: %s", why);
    invalidate_stream();
    unsigned char expected = 0;
    atomic_compare_exchange_strong(&s_cmd, &expected, 3);
}
static bool s_force_scan;       // rescan 后强制走扫描 (不直连)
static uint32_t s_scan_start_ms;

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static void set_phase(phase_t p) {
    s_ph = p;
    s_ph_ms = now_ms();
    switch (p) {
        case PH_SCAN:         s_ext_state = CUBE_CLIENT_SCAN; break;
        case PH_CONNECT_WAIT: s_ext_state = CUBE_CLIENT_CONNECTING; break;
        case PH_MTU:
        case PH_DISC_SVC:
        case PH_DISC_CHR:
        case PH_DISC_DSC:
        case PH_SUB_WRITE:    s_ext_state = CUBE_CLIENT_CONNECTING; break;
        case PH_SYNC:         s_ext_state = CUBE_CLIENT_SYNCING; break;
        case PH_RUN:          s_ext_state = CUBE_CLIENT_RUN; break;
        case PH_RECONN_WAIT:  s_ext_state = CUBE_CLIENT_RECONNECT; break;
        default:              s_ext_state = CUBE_CLIENT_STOPPED; break;
    }
}

// ===========================================================================
// 事件投递 (host 任务 -> 主循环)
// ===========================================================================
static void post_event(const cube_client_event_t *e) {
    if (atomic_load(&s_stream_fault)) return;
    cube_client_event_t copy = *e;
    copy.generation = atomic_load(&s_generation);
    if (!s_evtq || xQueueSend(s_evtq, &copy, 0) != pdTRUE) {
        stream_fault("event queue overflow");
    }
}

static void post_move(uint8_t face, uint8_t power, uint32_t timestamp_ms, uint8_t source) {
    cube_client_event_t e = {0};
    e.type = CUBE_EVT_MOVE;
    e.face = face;
    e.power = power;
    e.timestamp_ms = timestamp_ms;
    e.time_source = source;
    post_event(&e);
}

static void post_state(const char facelet[55]) {
    cube_state_t check;
    if (!cube_from_facelet(&check, facelet)) {
        stream_fault("invalid facelets");
        return;
    }
    cube_client_event_t e = {0};
    e.type = CUBE_EVT_STATE;
    memcpy(e.facelet, facelet, 54);
    e.facelet[54] = '\0';
    post_event(&e);
}

static void post_battery(uint8_t pct) {
    cube_client_event_t e = {0};
    e.type = CUBE_EVT_BATTERY;
    e.battery = pct;
    post_event(&e);
}

static void post_lost(void) {
    cube_client_event_t e = {0};
    e.type = CUBE_EVT_LOST;
    post_event(&e);
}

// 初始状态同步完成 -> RUN (保存配对)
static void sync_complete(void) {
    if (s_sync_done || atomic_load(&s_stream_fault)) return;
    s_sync_done = true;
    set_phase(PH_RUN);
    ESP_LOGI(TAG, "实体魔方 [%s] 同步完成, 进入 RUN (serial 续接由主循环维护)", s_cubename);
}

// ===========================================================================
// NVS 配对存储 (命名空间 "cubepair")
// ===========================================================================
#define NVS_NS "cubepair"

static void save_paired(const ble_addr_t *addr, cube_dev_type_t t, const uint8_t mac[6]) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    uint8_t buf[7];
    buf[0] = addr->type;
    memcpy(buf + 1, addr->val, 6);
    nvs_set_blob(h, "addr", buf, sizeof(buf));
    nvs_set_u8(h, "dtype", (uint8_t)t);
    nvs_set_blob(h, "mac", mac, 6);
    nvs_commit(h);
    nvs_close(h);
}

static bool load_paired(ble_addr_t *addr, cube_dev_type_t *t, uint8_t mac[6]) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    uint8_t buf[7];
    size_t len = sizeof(buf);
    uint8_t dtype = 0;
    bool ok = nvs_get_blob(h, "addr", buf, &len) == ESP_OK && len == sizeof(buf) &&
              nvs_get_u8(h, "dtype", &dtype) == ESP_OK && dtype != CUBE_DEV_NONE;
    if (ok) {
        addr->type = (uint8_t)buf[0];
        memcpy(addr->val, buf + 1, 6);
        *t = (cube_dev_type_t)dtype;
        len = 6;
        if (nvs_get_blob(h, "mac", mac, &len) != ESP_OK || len != 6) {
            memset(mac, 0, 6);   // MHC 无需 MAC
        }
    }
    nvs_close(h);
    return ok;
}

static void erase_paired(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
}

// ===========================================================================
// 通用写完成回调 (仅日志)
// ===========================================================================
static int write_done_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg) {
    if (error->status != 0 && error->status != BLE_HS_EDONE) {
        ESP_LOGW(TAG, "GATT 写入失败: status=%d attr=%d", error->status,
                 attr ? attr->handle : 0);
    }
    return 0;
}

static void send_write(uint16_t attr_h, const void *buf, uint16_t len) {
    int rc = ble_gattc_write_flat(s_conn, attr_h, buf, len, write_done_cb, NULL);
    if (rc != 0) ESP_LOGW(TAG, "ble_gattc_write_flat rc=%d", rc);
}

// ===========================================================================
// 协议: 魔域 MHC
// ===========================================================================
static struct {
    uint8_t face_status[6];
    cube_time_t clock;
    // v1 分片响应重组
    uint8_t parts[16][18];
    uint8_t partlen[16];
    bool has[16];
    uint8_t sendcnt;
    uint8_t reqid;
    bool state_rx;
} mhc;

static void mhc_reset(void) {
    memset(&mhc, 0, sizeof(mhc));
}

static void mhc_send_state_req(void) {
    uint8_t f[20] = {0};
    f[0] = mhc.sendcnt++;
    f[1] = 0x10;   // index=0, total=1
    f[2] = (10 & 0x0f) | ((mhc.reqid = (mhc.reqid + 1) & 7) << 5);
    send_write(s_write_h, f, sizeof(f));
    s_last_req_ms = now_ms();
}

// 解析 v1 command 10 的 30 字节状态载荷 -> URFDLB 面贴串
static void mhc_parse_state(const uint8_t *p) {
    static const char sid2col[7] = "DLBRFU";
    static const uint8_t cell2std[6][9] = {
        {27, 28, 29, 30, 31, 32, 33, 34, 35},
        {44, 43, 42, 41, 40, 39, 38, 37, 36},
        {53, 52, 51, 50, 49, 48, 47, 46, 45},
        {17, 16, 15, 14, 13, 12, 11, 10, 9},
        {26, 25, 24, 23, 22, 21, 20, 19, 18},
        {0, 1, 2, 3, 4, 5, 6, 7, 8},
    };
    char fl[55];
    for (int f = 0; f < 6; f++) {
        for (int c = 0; c < 9; c++) {
            int si = f * 9 + c;
            int v = (p[si / 2] >> ((si % 2) * 4)) & 0x0f;
            if (v >= 6) return;
            fl[cell2std[f][c]] = sid2col[v];
        }
    }
    fl[54] = '\0';
    for (int f = 0; f < 6; f++) {
        int a = (p[27 + f / 2] >> ((f % 2) * 4)) & 0x0f;
        mhc.face_status[f] = a % 9;
    }
    post_state(fl);
    mhc.state_rx = true;
    sync_complete();
}

static void mhc_on_read(const uint8_t *d, int len) {
    if (len < 2) return;
    int idx = d[1] & 0x0f;
    int total = d[1] >> 4;
    if (total <= 0 || idx >= 16) return;
    if (idx == 0) {
        memset(mhc.has, 0, sizeof(mhc.has));
    }
    int plen = len - 2;
    if (plen > 18) plen = 18;
    if (plen < 0) return;
    memcpy(mhc.parts[idx], d + 2, plen);
    mhc.partlen[idx] = (uint8_t)plen;
    mhc.has[idx] = true;
    for (int i = 0; i < total; i++) {
        if (!mhc.has[i]) return;
    }
    uint8_t merged[96];
    int n = 0;
    for (int i = 0; i < total && n < (int)sizeof(merged); i++) {
        int cp = mhc.partlen[i];
        if (n + cp > (int)sizeof(merged)) cp = (int)sizeof(merged) - n;
        memcpy(merged + n, mhc.parts[i], cp);
        n += cp;
        mhc.has[i] = false;
    }
    if (n < 1) return;
    uint8_t hdr = merged[0];
    int cmd = hdr & 0x0f;
    bool ok = ((hdr >> 4) & 1) == 1;
    int id = (hdr >> 5) & 7;
    if (!ok || cmd != 10 || id != mhc.reqid) return;
    if (n - 1 < 30) return;
    mhc_parse_state(merged + 1);
}

static void mhc_on_turn(const uint8_t *d, int len) {
    if (s_ph != PH_RUN) return;
    if (len < 1) { stream_fault("short MHC turn"); return; }
    int n = d[0];
    if (len < 1 + n * 6) { stream_fault("truncated MHC turn"); return; }
    for (int i = 0; i < n; i++) {
        if (d[1 + i * 6 + 4] > 5) { stream_fault("invalid MHC face"); return; }
    }
    for (int i = 0; i < n; i++) {
        const uint8_t *m = d + 1 + i * 6;
        int face = m[4];
        if (face > 5) continue;
        int8_t d8 = (int8_t)m[5];
        int dir = (d8 >= 0) ? (d8 + 18) / 36 : (d8 - 18) / 36;
        int prev = mhc.face_status[face];
        int cur = prev + dir;
        mhc.face_status[face] = (uint8_t)((cur + 9) % 9);
        int power = -1;
        if (prev >= 5 && cur <= 4) power = 1;       // CCW
        else if (prev <= 4 && cur >= 5) power = 0;  // CW
        if (power < 0) continue;                    // 未完成半圈
        // Source byte order: b1 b0 b3 b2, 65536 ticks per second.
        uint32_t raw = ((uint32_t)m[1] << 24) | ((uint32_t)m[0] << 16) |
                       ((uint32_t)m[3] << 8) | m[2];
        uint32_t ms;
        if (!cube_time_update(&mhc.clock, raw, 65536, &ms)) {
            stream_fault("MHC source time moved backwards");
            return;
        }
        post_move(MHC_AXIS_MAP[face], power, ms, CUBE_TIME_MHC);
    }
}

// ===========================================================================
// 协议: 魔域 MY3 (WCU_MY3)
// ===========================================================================
static struct {
    gan_crypto_t crypto;
    uint32_t device_ms;
    int move_cnt;
    int prev_move_cnt;
    bool facelet_rx;
    bool facelet_warned;   // 面贴无效告警只打一次
    uint8_t battery;
} my3;

static void my3_reset(void) {
    // 保留已派生密钥: 连接建立时也会调用本函数 (密钥在 connect_to 中生成)
    gan_crypto_t crypto = my3.crypto;
    memset(&my3, 0, sizeof(my3));
    my3.crypto = crypto;
    my3.prev_move_cnt = -1;
    my3.move_cnt = -1;
}

static void my3_send_req(uint8_t opcode) {
    uint8_t f[20] = {0};
    f[0] = opcode;
    gan_crypto_encode(&my3.crypto, f, sizeof(f));
    send_write(s_write_h, f, sizeof(f));
    s_last_req_ms = now_ms();
}

// 0xAC 开/关陀螺仪上报 (byte[2]=1/0, 基于作者私有 Flutter 项目移植)
static void my3_send_gyro(bool enable) {
    uint8_t f[20] = {0};
    f[0] = 0xAC;
    f[2] = enable ? 1 : 0;
    gan_crypto_encode(&my3.crypto, f, sizeof(f));
    send_write(s_write_h, f, sizeof(f));
    s_last_req_ms = now_ms();
}

// 大端位读取 (MSB first)
static uint32_t get_bits(const uint8_t *d, int bitpos, int nbits) {
    uint32_t v = 0;
    for (int i = 0; i < nbits; i++) {
        int bp = bitpos + i;
        v = (v << 1) | ((d[bp >> 3] >> (7 - (bp & 7))) & 1);
    }
    return v;
}

static void my3_on_facelet(const uint8_t *dec) {
    if (my3.prev_move_cnt >= 0) return;   // 已同步
    my3.move_cnt = (int)dec[19];          // bits 152-160
    static const uint8_t faces[6] = {2, 5, 0, 3, 4, 1};  // URFDLB -> 设备面序(FBUDLR)
    static const char dev_cols[7] = "FBUDLR";
    char fl[55];
    int pos = 0;
    for (int i = 0; i < 6; i++) {
        int f = faces[i];
        for (int j = 0; j < 8; j++) {
            int v = (int)get_bits(dec, 8 + f * 24 + j * 3, 3);
            if (v > 5) {
                if (!my3.facelet_warned) {   // 告警一次, 附解密原文便于排查
                    my3.facelet_warned = true;
                    char hex[3 * 20 + 1] = {0};
                    int h = 0;
                    for (int k = 0; k < 20; k++) {
                        snprintf(hex + h, 4, "%02X ", dec[k]);
                        h += 3;
                    }
                    ESP_LOGW(TAG, "[my3] 面贴含非法值 (face=%d j=%d v=%d), 解密帧: %s",
                             f, j, v, hex);
                }
                return;
            }
            fl[pos++] = dev_cols[v];
            if (j == 3) fl[pos++] = dev_cols[f];   // 中心块在第 4 贴后
        }
    }
    fl[54] = '\0';
    my3.prev_move_cnt = my3.move_cnt;
    my3.facelet_rx = true;
    post_state(fl);
    sync_complete();
}

static void my3_on_moves(const uint8_t *dec) {
    my3.move_cnt = (int)dec[11];          // bits 88-96
    if (my3.prev_move_cnt < 0) {
        // 面贴未同步: 重新请求 (节流)
        if (now_ms() - s_last_req_ms > 2000) {
            my3_send_req(163);
        }
        return;
    }
    if (my3.move_cnt == my3.prev_move_cnt) return;

    uint8_t mv[5];
    bool invalid = false;
    for (int i = 0; i < 5; i++) {
        mv[i] = (uint8_t)get_bits(dec, 96 + i * 5, 5);
        if (mv[i] >= 12) invalid = true;
    }
    int diff = (my3.move_cnt - my3.prev_move_cnt) & 0xff;
    if (invalid) {
        stream_fault("MY3 invalid move encoding");
        return;
    }
    if (diff > 5) {
        stream_fault("MY3 gap exceeds five-move history");
        return;
    }
    my3.prev_move_cnt = my3.move_cnt;
    // 槽位 0 = 最新, 从最旧的应用起
    for (int i = diff - 1; i >= 0; i--) {
        uint8_t m = mv[i];
        const char *p = strchr("URFDLB", "FBUDLR"[m >> 1]);
        if (!p) continue;
        my3.device_ms += get_bits(dec, 8 + i * 16, 16);
        post_move((uint8_t)(p - "URFDLB"), m & 1, my3.device_ms, CUBE_TIME_MY3);
    }
}

static void my3_on_notify(const uint8_t *d, int len) {
    if (len != 20) { stream_fault("invalid MY3 frame length"); return; }
    uint8_t dec[20];
    memcpy(dec, d, 20);
    gan_crypto_decode(&my3.crypto, dec, sizeof(dec));
    if (dec[0] != 171) {   // 0xAB 陀螺仪帧不打印, 避免刷屏
        ESP_LOGI(TAG, "[my3] 解密首字节=%d len=%d (161=info 163=facelet 164=bat 165=move)",
                 dec[0], len);
    }
    switch (dec[0]) {
        case 163: my3_on_facelet(dec); break;
        case 164:
            if (dec[1] != my3.battery) {
                my3.battery = dec[1];
                post_battery(my3.battery);
            }
            break;
        case 165: my3_on_moves(dec); break;
        default: break;   // 161 信息 / 171 陀螺仪 / 未知(解密失败)
    }
}

// ===========================================================================
// 协议: 奇艺
// ===========================================================================
static struct {
    uint32_t last_ts;
    bool has_ts;
    cube_time_t clock;
    bool hello_rx;
    bool state_rx;
    uint8_t battery;
    cube_state_t shadow;   // 校验用影子状态
} qy;

static void qy_reset(void) {
    memset(&qy, 0, sizeof(qy));
    cube_init(&qy.shadow);
}

static uint16_t crc16_modbus(const uint8_t *d, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= d[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
        }
    }
    return crc;
}

static void qy_aes(uint8_t *out, const uint8_t *in, int len, bool encrypt) {
    esp_aes_context ctx;
    esp_aes_init(&ctx);
    esp_aes_setkey(&ctx, QY_AES_KEY, 128);
    for (int i = 0; i + 16 <= len; i += 16) {
        esp_aes_crypt_ecb(&ctx, encrypt ? ESP_AES_ENCRYPT : ESP_AES_DECRYPT,
                          in + i, out + i);
    }
    esp_aes_free(&ctx);
}

// 组帧+加密+写入: content = opcode+载荷
static void qy_send(const uint8_t *content, int len) {
    uint8_t m[48];
    int n = 0;
    m[n++] = 0xFE;
    m[n++] = (uint8_t)(4 + len);
    memcpy(m + n, content, len);
    n += len;
    uint16_t crc = crc16_modbus(m, n);
    m[n++] = crc & 0xff;
    m[n++] = crc >> 8;
    int pad = (16 - n % 16) % 16;
    memset(m + n, 0, pad);
    n += pad;
    qy_aes(m, m, n, true);
    send_write(s_write_h, m, n);
}

static void qy_send_hello(void) {
    uint8_t content[17] = {0x00, 0x6b, 0x01, 0x00, 0x00, 0x22, 0x06, 0x00,
                           0x02, 0x08, 0x00};
    // MAC 显示序逆序拼接
    for (int i = 5; i >= 0; i--) {
        content[11 + (5 - i)] = s_cubemac[i];
    }
    qy_send(content, sizeof(content));
    s_last_req_ms = now_ms();
}

static bool qy_parse_facelet(const uint8_t *p, char out[55]) {
    static const char cols[7] = "LRDUFB";
    for (int i = 0; i < 54; i++) {
        int v = (p[i >> 1] >> ((i % 2) << 2)) & 0x0f;
        if (v > 5) return false;
        out[i] = cols[v];
    }
    out[54] = '\0';
    return true;
}

static uint32_t be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

// 回绕感知的 32 位时间戳比较: candidate 是否严格晚于 reference
static bool ts_after(uint32_t cand_ts, uint32_t ref) {
    uint32_t d = cand_ts - ref;
    return d != 0 && d < 0x80000000;
}

// hello (0x02) / 同步确认 (0x04): 携带完整面贴+电量
static void qy_on_state_frame(uint8_t *msg, int msg_len) {
    if (msg_len < 38) { stream_fault("short QY state"); return; }
    qy_send(msg + 2, 5);
    char fl[55];
    cube_state_t state;
    if (!qy_parse_facelet(msg + 7, fl) || !cube_from_facelet(&state, fl)) {
        stream_fault("invalid QY state");
        return;
    }
    if (qy.state_rx && memcmp(&state, &qy.shadow, sizeof(state)) != 0) {
        stream_fault("QY state changed without moves");
        return;
    }
    qy.hello_rx = qy.state_rx = true;
    qy.shadow = state;
    qy.last_ts = be32(msg + 3);
    qy.has_ts = true;
    uint32_t unused;
    if (!cube_time_update(&qy.clock, qy.last_ts, 1600, &unused)) {
        stream_fault("QY state clock moved backwards");
        return;
    }
    post_state(fl);
    sync_complete();
    qy.battery = msg[35];
    post_battery(qy.battery);
}

// 状态变化 (0x03): 当前动作 + 11 个历史槽, 时间戳恢复 + 面贴校验
static void qy_on_change(uint8_t *msg, int msg_len) {
    if (msg_len < 38) { stream_fault("short QY change"); return; }
    qy_send(msg + 2, 5);   // ACK
    if (!qy.state_rx) return; // Initial absolute state is mandatory.
    qy.hello_rx = true;

    typedef struct {
        uint8_t raw;
        uint32_t ts;
        uint8_t order;
        bool primary;
    } qcand_t;
    qcand_t cand[12];
    int nc = 0;
    uint32_t frame_ts = be32(msg + 3);
    uint32_t last_ts = qy.last_ts;

    // 当前动作 (允许与上一动作同时间戳: 中层转动拆分)
    uint8_t pr = msg[34];
    if (pr >= 1 && pr <= 12 &&
        (!qy.has_ts || frame_ts == last_ts || ts_after(frame_ts, last_ts))) {
        cand[nc].raw = pr; cand[nc].ts = frame_ts;
        cand[nc].order = 11; cand[nc].primary = true; nc++;
    }
    // 历史槽: 5 字节 (时间戳4 + 编码1)
    for (int slot = 0; slot < 11; slot++) {
        int off = 36 + slot * 5;
        if (off + 5 > msg_len - 2) break; // exclude CRC
        bool empty = true;
        for (int i = 0; i < 5; i++) {
            if (msg[off + i] != 0xff) { empty = false; break; }
        }
        if (empty) continue;
        uint8_t raw = msg[off + 4];
        if (raw < 1 || raw > 12) continue;
        uint32_t t = be32(msg + off);
        if (qy.has_ts && !ts_after(t, last_ts)) continue;
        if (ts_after(t, frame_ts)) continue;
        if (nc < 12) {
            cand[nc].raw = raw; cand[nc].ts = t;
            cand[nc].order = (uint8_t)slot; cand[nc].primary = false; nc++;
        }
    }
    // 排序: 时间戳升序, 相同则 order 升序 (插入排序)
    for (int i = 1; i < nc; i++) {
        qcand_t k = cand[i];
        int j = i - 1;
        while (j >= 0 &&
               (ts_after(cand[j].ts, k.ts) || (cand[j].ts == k.ts && cand[j].order > k.order))) {
            cand[j + 1] = cand[j];
            j--;
        }
        cand[j + 1] = k;
    }
    // 去掉当前动作在历史环中的最新镜像
    int mirror = -1;
    for (int i = 0; i < nc; i++) {
        if (cand[i].primary) {
            for (int j = 0; j < nc; j++) {
                if (!cand[j].primary && cand[j].raw == cand[i].raw &&
                    cand[j].ts == cand[i].ts) {
                    mirror = j;
                }
            }
        }
    }
    // Validate the complete batch before publishing any of it to the main task.
    char fl_frame[55];
    cube_state_t target, next = qy.shadow;
    if (!qy_parse_facelet(msg + 7, fl_frame) || !cube_from_facelet(&target, fl_frame)) {
        stream_fault("invalid QY change facelets");
        return;
    }
    if (qy.has_ts && frame_ts == last_ts &&
        memcmp(&target, &qy.shadow, sizeof(target)) == 0) return; // retransmission
    if (qy.has_ts && ts_after(last_ts, frame_ts)) return; // stale notification
    uint32_t move_ms[12] = {0};
    cube_time_t clock = qy.clock;
    for (int i = 0; i < nc; i++) {
        if (i == mirror) continue;
        uint8_t raw = cand[i].raw;
        cube_move(&next, QY_AXIS_MAP[(raw - 1) >> 1], (raw & 1) ? 2 : 0);
        if (!cube_time_update(&clock, cand[i].ts, 1600, &move_ms[i])) {
            stream_fault("QY source time moved backwards");
            return;
        }
    }
    if (memcmp(&next, &target, sizeof(next)) != 0) {
        stream_fault("QY history cannot reconstruct facelets");
        return;
    }
    qy.clock = clock;
    qy.shadow = next;
    qy.last_ts = frame_ts;
    qy.has_ts = true;
    for (int i = 0; i < nc; i++) {
        if (i == mirror) continue;
        uint8_t raw = cand[i].raw;
        post_move(QY_AXIS_MAP[(raw - 1) >> 1], raw & 1, move_ms[i], CUBE_TIME_QIYI);
    }
    if (msg[35] != qy.battery) {
        qy.battery = msg[35];
        post_battery(qy.battery);
    }
}

static void qy_on_notify(const uint8_t *d, int len) {
    if (len <= 0 || len % 16 != 0 || len > QY_MAX_FRAME) return;
    uint8_t msg[QY_MAX_FRAME];
    qy_aes(msg, d, len, false);
    if (msg[0] == 0xCC) return; // Optional gyro frames do not alter turn history.
    int msg_len = msg[1];
    if (msg_len < 9 || msg_len > len) { stream_fault("invalid QY length"); return; }
    if (crc16_modbus(msg, msg_len) != 0) { stream_fault("QY CRC"); return; }
    if (msg[0] != 0xFE) return;
    switch (msg[2]) {
        case 0x02:
        case 0x04: qy_on_state_frame(msg, msg_len); break;
        case 0x03: qy_on_change(msg, msg_len); break;
        default: break;
    }
}

// ===========================================================================
// 扫描候选管理
// ===========================================================================
static cand_t *cand_find(const ble_addr_t *addr) {
    for (int i = 0; i < 4; i++) {
        if (s_cands[i].type != CUBE_DEV_NONE || s_cands[i].mfg_len > 0) {
            if (s_cands[i].addr.type == addr->type &&
                memcmp(s_cands[i].addr.val, addr->val, 6) == 0) {
                return &s_cands[i];
            }
        }
    }
    return NULL;
}

static cand_t *cand_alloc(const ble_addr_t *addr) {
    cand_t *c = cand_find(addr);
    if (c) return c;
    // 找空槽 (未识别且无厂商数据)
    for (int i = 0; i < 4; i++) {
        if (s_cands[i].type == CUBE_DEV_NONE && s_cands[i].mfg_len == 0) {
            memset(&s_cands[i], 0, sizeof(cand_t));
            s_cands[i].addr = *addr;
            return &s_cands[i];
        }
    }
    return NULL;
}

static bool hex_nibble(char c) {
    return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
}

static int parse_hex2(const char *s) {
    int hi, lo;
    if (s[0] >= '0' && s[0] <= '9') hi = s[0] - '0';
    else if (s[0] >= 'A' && s[0] <= 'F') hi = s[0] - 'A' + 10;
    else if (s[0] >= 'a' && s[0] <= 'f') hi = s[0] - 'a' + 10;
    else return -1;
    if (s[1] >= '0' && s[1] <= '9') lo = s[1] - '0';
    else if (s[1] >= 'A' && s[1] <= 'F') lo = s[1] - 'A' + 10;
    else if (s[1] >= 'a' && s[1] <= 'f') lo = s[1] - 'a' + 10;
    else return -1;
    return hi * 16 + lo;
}

// 名称前缀分类 (不依赖候选, 供候选槽预筛)
static cube_dev_type_t classify_name(const char *name) {
    if (strncasecmp(name, "MHC", 3) == 0) return CUBE_DEV_MOYU_MHC;
    if (strncasecmp(name, "WCU_MY3", 7) == 0) return CUBE_DEV_MOYU_MY3;
    if (strncasecmp(name, "QY-QYSC", 7) == 0 ||
        strncasecmp(name, "XMD-TornadoV4-i", 15) == 0) {
        return CUBE_DEV_QIYI;
    }
    return CUBE_DEV_NONE;
}

// 名称识别 + 名称后缀 MAC 兜底
static cube_dev_type_t match_name(cand_t *c, const char *name) {
    cube_dev_type_t t = classify_name(name);
    if (t == CUBE_DEV_NONE || t == CUBE_DEV_MOYU_MHC) return t;   // MHC 无需 MAC
    // 名称后 4 位十六进制 -> 默认 MAC 前缀兜底 (权威 MAC 以厂商数据 CIC 为准)
    size_t n = strlen(name);
    if (!c->has_mac && n >= 4 && hex_nibble(name[n-4]) && hex_nibble(name[n-3]) &&
        hex_nibble(name[n-2]) && hex_nibble(name[n-1])) {
        int hh = parse_hex2(name + n - 4);
        int ll = parse_hex2(name + n - 2);
        if (hh >= 0 && ll >= 0) {
            if (t == CUBE_DEV_MOYU_MY3) {
                // WCU_MY3: 默认 MAC CF:30:16:00:XX:XX
                c->mac[0] = 0xCF; c->mac[1] = 0x30; c->mac[2] = 0x16;
                c->mac[3] = 0x00; c->mac[4] = (uint8_t)hh; c->mac[5] = (uint8_t)ll;
            } else {
                // QY-QYSC / XMD: 默认 MAC CC:A3:00:00:XX:XX
                c->mac[0] = 0xCC; c->mac[1] = 0xA3; c->mac[2] = 0x00;
                c->mac[3] = 0x00; c->mac[4] = (uint8_t)hh; c->mac[5] = (uint8_t)ll;
            }
            c->has_mac = true;
        }
    }
    return t;
}

// 根据已缓存厂商数据派生 MAC (名称与厂商数据可能分帧到达)
// 注意: 厂商数据里的 CIC MAC 才是权威来源 (用于派生密钥/握手),
// 必须优先于名称后缀猜测值, 因此这里不因 has_mac 提前返回
static void update_cand_mac(cand_t *c) {
    const uint8_t *d = c->mfg;
    int len = c->mfg_len;
    if (c->type == CUBE_DEV_QIYI && len >= 8 && d[0] == 0x04 && d[1] == 0x05) {
        // CIC 0x0504: 前 6 字节为 MAC, 显示序 = 逆序
        for (int k = 0; k < 6; k++) c->mac[k] = d[2 + 5 - k];
        c->has_mac = true;
    } else if (c->type == CUBE_DEV_MOYU_MY3 && len >= 8) {
        // MY3 实测厂商数据: 00 00 00 00 30 0D 57 02 16 30 CF
        // MAC 恒为末 6 字节逆序 (显示序), 不再要求固定头部格式
        for (int k = 0; k < 6; k++) c->mac[k] = d[len - 1 - k];
        c->has_mac = true;
    } else if ((c->type == CUBE_DEV_MOYU_MY3 || c->type == CUBE_DEV_QIYI) && len < 8) {
        // 厂商数据不足 8 字节: 用实际 BLE 地址 (显示序) 兜底
        for (int k = 0; k < 6; k++) c->mac[k] = c->addr.val[5 - k];
        c->has_mac = true;
    }
}

// 调试: 打印最终选中 MAC 与 BLE 地址对比, 便于排查密钥派生
static void debug_dump_mac(const char *tag, const ble_addr_t *addr, const uint8_t mac[6]) {
    char a[18], m[18];
    snprintf(a, sizeof(a), "%02X:%02X:%02X:%02X:%02X:%02X",
             addr->val[0], addr->val[1], addr->val[2],
             addr->val[3], addr->val[4], addr->val[5]);
    snprintf(m, sizeof(m), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "[mac] %s: BLE=%s CIC/派生=%s", tag, a, m);
}

static void handle_disc(struct ble_gap_event *event) {
    const struct ble_gap_disc_desc *disc = &event->disc;
    struct ble_hs_adv_fields f;
    if (ble_hs_adv_parse_fields(&f, disc->data, disc->length_data) != 0) return;

    // ---- 先解析本帧, 收集魔方线索 ----
    char name[24] = {0};
    if (f.name && f.name_len > 0) {
        int n = f.name_len < (int)sizeof(name) - 1 ? f.name_len : (int)sizeof(name) - 1;
        memcpy(name, f.name, n);
    }
    bool mhc_uuid = false;
    if (f.uuids16 && f.num_uuids16 > 0) {
        for (int i = 0; i < f.num_uuids16; i++) {
            if (ble_uuid_u16((const ble_uuid_t *)&f.uuids16[i]) == MHC_SVC_UUID) {
                mhc_uuid = true;
                break;
            }
        }
    }
    // 奇艺 CIC 0x0504: 前 6 字节为 MAC
    bool qy_cic = f.mfg_data && f.mfg_data_len >= 8 &&
                  f.mfg_data[0] == 0x04 && f.mfg_data[1] == 0x05;
    cube_dev_type_t t_name = classify_name(name);

    // ---- 候选槽只给魔方广播 ----
    // 环境中手机/耳机等大量设备携带厂商数据广播, 若先占槽后识别,
    // 4 个候选位被无关设备耗尽后魔方将永远无法被识别 (广播会持续重复,
    // 名称/UUID/CIC 任一帧到达即可占槽, 其余线索帧经 cand_find 合并)
    cand_t *c = cand_find(&disc->addr);
    if (c == NULL) {
        if (t_name == CUBE_DEV_NONE && !mhc_uuid && !qy_cic) return;
        c = cand_alloc(&disc->addr);
        if (!c) return;
    }
    c->rssi = disc->rssi;

    if (name[0] != '\0') {
        memcpy(c->name, name, strlen(name) + 1);
        cube_dev_type_t t = match_name(c, c->name);
        if (t != CUBE_DEV_NONE) c->type = t;
    }
    if (mhc_uuid && c->type == CUBE_DEV_NONE) c->type = CUBE_DEV_MOYU_MHC;
    if (f.mfg_data && f.mfg_data_len > 0) {
        bool first_mfg = (c->mfg_len == 0);
        int n = f.mfg_data_len < (int)sizeof(c->mfg) ? f.mfg_data_len : (int)sizeof(c->mfg);
        memcpy(c->mfg, f.mfg_data, n);
        c->mfg_len = (uint8_t)n;
        if (qy_cic && c->type == CUBE_DEV_NONE) c->type = CUBE_DEV_QIYI;
        // 调试: 首次收到厂商数据时打印原始字节, 便于核对 CIC/MAC 解析
        if (first_mfg && (c->type == CUBE_DEV_MOYU_MY3 || c->type == CUBE_DEV_QIYI)) {
            char hex[3 * sizeof(c->mfg) + 1] = {0};
            int h = 0;
            for (int i = 0; i < n; i++) {
                snprintf(hex + h, 4, "%02X ", c->mfg[i]);
                h += 3;
            }
            ESP_LOGI(TAG, "[mfg] type=%d len=%d data=%s", (int)c->type, n, hex);
        }
    }
    update_cand_mac(c);

    // 首次识别打印: 无此日志说明从未收到魔方广播 (魔方可能休眠/已被其他App连接)
    if (c->type != CUBE_DEV_NONE && !c->id_logged) {
        c->id_logged = true;
        ESP_LOGI(TAG, "[disc] 识别到魔方广播: type=%d name=%s rssi=%d",
                 (int)c->type, c->name[0] ? c->name : "(uuid/cic)", c->rssi);
    }
}

// ===========================================================================
// 连接失败/超时兜底
// ===========================================================================
static void start_scan(void);
static void try_direct_connect(void);

static void fail_and_rescan(const char *why) {
    invalidate_stream();
    ESP_LOGW(TAG, "%s -> 重新扫描", why);
    if (s_conn != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
        s_force_scan = true;   // 断开事件后走扫描
        set_phase(PH_RECONN_WAIT);
    } else {
        start_scan();
    }
}

// ===========================================================================
// GATT 发现与订阅链 (回调链互相引用, 统一前置声明)
// ===========================================================================
static int disc_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg);
static void begin_subscribe(void);
static void write_cccd(void);
static int write_cccd_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg);
static void on_all_subscribed(void);
static void disc_next_dscs(void);
static int disc_dsc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc,
                       void *arg);

static int disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *svc, void *arg) {
    if (conn_handle != s_conn) return 0;
    if (error->status == 0 && svc != NULL) {
        s_svc_start = svc->start_handle;
        s_svc_end = svc->end_handle;
        return 0;
    }
    if (error->status == BLE_HS_EDONE) {
        if (s_svc_start == 0) {
            fail_and_rescan("目标服务未找到");
            return 0;
        }
        s_chr_n = 0;
        set_phase(PH_DISC_CHR);
        int rc = ble_gattc_disc_all_chrs(s_conn, s_svc_start, s_svc_end,
                                          disc_chr_cb, NULL);
        if (rc != 0) fail_and_rescan("特征发现启动失败");
        return 0;
    }
    fail_and_rescan("服务发现错误");
    return 0;
}

static int disc_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg) {
    if (conn_handle != s_conn) return 0;
    if (error->status == 0 && chr != NULL) {
        if (s_chr_n < 12) {
            s_chrs[s_chr_n].def_h = chr->def_handle;
            s_chrs[s_chr_n].val_h = chr->val_handle;
            s_chrs[s_chr_n].uuid = chr->uuid;   // IDF6.0: uuid 为内嵌值
            s_chr_n++;
        }
        return 0;
    }
    if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "特征发现完成: n=%d", s_chr_n);
        for (int i = 0; i < s_chr_n; i++) {
            ESP_LOGI(TAG, "  chr[%d] def=%d val=%d", i, s_chrs[i].def_h, s_chrs[i].val_h);
        }
        begin_subscribe();
        return 0;
    }
    fail_and_rescan("特征发现错误");
    return 0;
}

// 按类型查特征句柄
static const chr_rec_t *find_chr(const ble_uuid_t *want) {
    for (int i = 0; i < s_chr_n; i++) {
        if (ble_uuid_cmp((const ble_uuid_t *)&s_chrs[i].uuid, want) == 0) {
            return &s_chrs[i];
        }
    }
    return NULL;
}

// 特征发现完成: 按协议挑出 写特征 + 通知特征, 开始 CCCD 订阅链
static void begin_subscribe(void) {
    const chr_rec_t *wr = NULL, *nt[3] = {NULL, NULL, NULL};
    int nn = 0;
    switch (s_devtype) {
        case CUBE_DEV_MOYU_MHC:
            wr = find_chr((const ble_uuid_t *)BLE_UUID16_DECLARE(MHC_CHR_WRITE));
            nt[0] = find_chr((const ble_uuid_t *)BLE_UUID16_DECLARE(MHC_CHR_READ));
            nt[1] = find_chr((const ble_uuid_t *)BLE_UUID16_DECLARE(MHC_CHR_TURN));
            nn = 2;
            break;
        case CUBE_DEV_MOYU_MY3:
            wr = find_chr((const ble_uuid_t *)&MY3_CHR_WRITE_UUID);
            nt[0] = find_chr((const ble_uuid_t *)&MY3_CHR_READ_UUID);
            nn = 1;
            break;
        case CUBE_DEV_QIYI:
            wr = find_chr((const ble_uuid_t *)BLE_UUID16_DECLARE(QY_CHR_UUID));
            nt[0] = wr;
            nn = 1;
            break;
        default:
            break;
    }
    if (wr == NULL) {
        fail_and_rescan("写特征未找到");
        return;
    }
    s_write_h = wr->val_h;
    for (int i = 0; i < nn; i++) {
        if (nt[i] == NULL) {
            fail_and_rescan("通知特征未找到");
            return;
        }
    }
    ESP_LOGI(TAG, "订阅: 写特征 val=%d, 通知特征数=%d", s_write_h, nn);
    // 构建通知目标列表 + 描述符搜索区间
    s_notify_n = 0;
    for (int i = 0; i < nn; i++) {
        s_notify[s_notify_n].val_h = nt[i]->val_h;
        // 下一特征 def_handle - 1, 最后一个用服务 end
        const chr_rec_t *next = NULL;
        for (int j = 0; j < s_chr_n; j++) {
            if (s_chrs[j].def_h > nt[i]->val_h &&
                (next == NULL || s_chrs[j].def_h < next->def_h)) {
                next = &s_chrs[j];
            }
        }
        s_notify[s_notify_n].next_def =
            next ? next->def_h : (uint16_t)(s_svc_end + 1);
        s_notify_n++;
    }
    s_sub_i = 0;
    set_phase(PH_DISC_DSC);
    disc_next_dscs();
}

static void write_cccd(void) {
    uint16_t val = 0x0001;
    uint16_t h = s_cccd_h ? s_cccd_h : (uint16_t)(s_notify[s_sub_i].val_h + 1);
    int rc = ble_gattc_write_flat(s_conn, h, &val, sizeof(val),
                                  write_cccd_cb, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "CCCD 写入启动失败 rc=%d, 假定 val+1 重试", rc);
        fail_and_rescan("CCCD 写入失败");
    }
}

static int write_cccd_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg) {
    if (conn_handle != s_conn) return 0;
    if (error->status != 0 && error->status != BLE_HS_EDONE) {
        fail_and_rescan("CCCD 写入错误");
        return 0;
    }
    s_sub_i++;
    if (s_sub_i < s_notify_n) {
        disc_next_dscs();
    } else {
        on_all_subscribed();
    }
    return 0;
}

static void disc_next_dscs(void) {
    s_cccd_h = 0;
    uint16_t start = (uint16_t)(s_notify[s_sub_i].val_h + 1);
    uint16_t end = (uint16_t)(s_notify[s_sub_i].next_def - 1);
    if (start <= end) {
        int rc = ble_gattc_disc_all_dscs(s_conn, start, end, disc_dsc_cb, NULL);
        if (rc != 0) write_cccd();   // 发现失败: 退化为 val+1 假定
    } else {
        write_cccd();
    }
}

static int disc_dsc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc,
                       void *arg) {
    if (conn_handle != s_conn) return 0;
    if (error->status == 0 && dsc != NULL) {
        if (ble_uuid_u16((const ble_uuid_t *)&dsc->uuid) ==
            BLE_GATT_DSC_CLT_CFG_UUID16) {
            s_cccd_h = dsc->handle;
        }
        return 0;
    }
    if (error->status == BLE_HS_EDONE) {
        write_cccd();
        return 0;
    }
    write_cccd();   // 出错仍尝试假定句柄
    return 0;
}

// 全部通知订阅完成: 协议初始化
static void on_all_subscribed(void) {
    set_phase(PH_SYNC);
    s_sync_retries = 0;
    switch (s_devtype) {
        case CUBE_DEV_MOYU_MHC:
            mhc_send_state_req();
            break;
        case CUBE_DEV_MOYU_MY3:
            my3_send_req(161);   // 0xA1 Cube Info 初始化 (连接后必须先发一次)
            my3_send_req(163);   // 请求面贴状态
            my3_send_req(164);   // 请求电量
            my3_send_gyro(false);   // 0xAC 关闭陀螺仪上报 (网关不需要, 避免高频通知)
            break;
        case CUBE_DEV_QIYI:
            qy_send_hello();
            break;
        default:
            break;
    }
}

static void start_disc_svc(void) {
    s_svc_start = s_svc_end = 0;
    set_phase(PH_DISC_SVC);
    const ble_uuid_t *svc_uuid;
    switch (s_devtype) {
        case CUBE_DEV_MOYU_MHC:
            svc_uuid = (const ble_uuid_t *)&MHC_SVC_UUID16;
            break;
        case CUBE_DEV_MOYU_MY3:
            svc_uuid = (const ble_uuid_t *)&MY3_SVC_UUID;
            break;
        case CUBE_DEV_QIYI:
            svc_uuid = (const ble_uuid_t *)&QY_SVC_UUID16;
            break;
        default:
            fail_and_rescan("未知设备类型");
            return;
    }
    int rc = ble_gattc_disc_svc_by_uuid(s_conn, svc_uuid, disc_svc_cb, NULL);
    if (rc != 0) fail_and_rescan("服务发现启动失败");
}

// ===========================================================================
// Central GAP 事件
// ===========================================================================
static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_DISC:
            if (s_ph == PH_SCAN) handle_disc(event);
            return 0;

        case BLE_GAP_EVENT_DISC_COMPLETE:
            if (s_ph == PH_SCAN) {
                ESP_LOGW(TAG, "扫描意外结束, 重启扫描");
                start_scan();
            }
            return 0;

        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status != 0) {
                ESP_LOGW(TAG, "连接失败 status=%d -> %s",
                         event->connect.status,
                         s_have_last_addr && !s_force_scan ? "重试直连" : "扫描");
                if (s_have_last_addr && !s_force_scan) {
                    set_phase(PH_RECONN_WAIT);   // tick 中直连重试
                } else {
                    start_scan();
                }
                return 0;
            }
            s_conn = event->connect.conn_handle;
            atomic_fetch_add(&s_generation, 1);
            atomic_store(&s_stream_fault, false);
            s_sync_done = false;
            s_notify_log_n = 0;
            // 每次新连接重置协议状态: 静态区上电为 0 而非 -1 哨兵,
            // 不重置会导致开机后首次 MY3 连接被当作"已同步"而超时
            mhc_reset();
            my3_reset();
            qy_reset();
            ESP_LOGI(TAG, "已连接 [%s] conn=%d, MTU 协商...", s_cubename, s_conn);
            set_phase(PH_MTU);
            {
                int rc = ble_gattc_exchange_mtu(s_conn, NULL, NULL);
                if (rc != 0) start_disc_svc();   // 协商启动失败: 直接发现
            }
            return 0;

        case BLE_GAP_EVENT_MTU:
            if (event->mtu.conn_handle == s_conn && s_ph == PH_MTU) {
                ESP_LOGI(TAG, "MTU=%d", event->mtu.value);
                start_disc_svc();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT: {
            if (event->disconnect.conn.conn_handle != s_conn) return 0;
            ESP_LOGW(TAG, "实体魔方断开 reason=%d", event->disconnect.reason);
            invalidate_stream();
            s_conn = BLE_HS_CONN_HANDLE_NONE;
            // 保留 s_last_addr / s_cubemac 供重连
            mhc_reset();
            my3_reset();
            qy_reset();
            s_sync_done = false;
            s_write_h = 0;
            s_notify_n = 0;
            s_chr_n = 0;
            post_lost();
            set_phase(PH_RECONN_WAIT);
            return 0;
        }

        case BLE_GAP_EVENT_NOTIFY_RX: {
            if (event->notify_rx.conn_handle != s_conn) return 0;
            if (s_ph != PH_SYNC && s_ph != PH_RUN) return 0;
            if (atomic_load(&s_stream_fault)) return 0;
            uint8_t data[QY_MAX_FRAME];
            int len = OS_MBUF_PKTLEN(event->notify_rx.om);
            uint16_t copied;
            if (len > sizeof(data) || ble_hs_mbuf_to_flat(event->notify_rx.om,
                    data, sizeof(data), &copied) != 0 || copied != len) {
                stream_fault("invalid or fragmented notification");
                return 0;
            }
            const uint8_t *d = data;
            uint16_t attr = event->notify_rx.attr_handle;
            // 逐帧十六进制日志仅打印每连接前几帧 (握手调试), 高频通知不再刷屏
            if (s_notify_log_n < 8) {
                s_notify_log_n++;
                char hex[3 * 32 + 1] = {0};
                int h = 0;
                for (int i = 0; i < len && i < 32; i++) {
                    snprintf(hex + h, 4, "%02X ", d[i]);
                    h += 3;
                }
                ESP_LOGI(TAG, "[notify] attr=%d len=%d hex=%s", attr, len, hex);
            }
            switch (s_devtype) {
                case CUBE_DEV_MOYU_MHC:
                    if (attr == s_notify[0].val_h) mhc_on_read(d, len);
                    else if (s_notify_n > 1 && attr == s_notify[1].val_h) mhc_on_turn(d, len);
                    break;
                case CUBE_DEV_MOYU_MY3:
                    if (s_notify_n > 0 && attr == s_notify[0].val_h) my3_on_notify(d, len);
                    break;
                case CUBE_DEV_QIYI:
                    if (s_notify_n > 0 && attr == s_notify[0].val_h) qy_on_notify(d, len);
                    break;
                default:
                    break;
            }
            return 0;
        }

        default:
            return 0;
    }
}

// ===========================================================================
// 扫描 / 直连
// ===========================================================================
static void start_scan(void) {
    ble_gap_disc_cancel();   // 忽略返回值
    memset(s_cands, 0, sizeof(s_cands));
    struct ble_gap_disc_params dp = {
        .itvl = 80,            // 50ms 间隔
        .window = 40,         // 25ms 窗口 (50% 占空比, 兼顾省电与发现速度)
        .filter_policy = 0,
        .limited = 0,
        .passive = 0,         // 主动扫描 (获取扫描响应中的名称)
        .filter_duplicates = 0,
    };
    int rc = ble_gap_disc(BLE_OWN_ADDR_RANDOM, BLE_HS_FOREVER, &dp,
                          gap_event_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "扫描启动失败 rc=%d", rc);
        return;
    }
    s_scan_start_ms = now_ms();
    set_phase(PH_SCAN);
    ESP_LOGI(TAG, "开始扫描实体魔方 (MHC / WCU_MY3 / QY-QYSC)...");
}

static void connect_to(const ble_addr_t *addr, cube_dev_type_t t,
                       const uint8_t mac[6], bool has_mac, const char *name) {
    set_phase(PH_CONNECT_WAIT);
    s_devtype = t;
    memcpy(s_cubemac, mac, 6);
    (void)has_mac;
    snprintf(s_cubename, sizeof(s_cubename), "%s", name ? name : "unknown");
    s_last_addr = *addr;
    s_have_last_addr = true;
    if (t == CUBE_DEV_MOYU_MY3) {
        gan_crypto_init_base(&my3.crypto, MY3_KEY_BASE, MY3_IV_BASE, s_cubemac);
    }
    // 扫描中先停止扫描再连接
    ble_gap_disc_cancel();
    int rc = ble_gap_connect(BLE_OWN_ADDR_RANDOM, addr, 10000, NULL,
                            gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_gap_connect rc=%d -> 重新扫描", rc);
        start_scan();
    }
}

static void try_direct_connect(void) {
    if (!s_have_last_addr) {
        start_scan();
        return;
    }
    connect_to(&s_last_addr, s_devtype, s_cubemac, true, s_cubename);
}

static void pick_best_and_connect(void) {
    cand_t *best = NULL;
    for (int i = 0; i < 4; i++) {
        cand_t *c = &s_cands[i];
        if (c->type == CUBE_DEV_NONE) continue;
        // MY3/QY 需要MAC (密钥/hello); MHC 无需
        if (c->type != CUBE_DEV_MOYU_MHC && !c->has_mac) continue;
        if (best == NULL || c->rssi > best->rssi) best = c;
    }
    if (best == NULL) return;   // 继续扫描
    char macstr[18] = {0};
    if (best->has_mac) {
        snprintf(macstr, sizeof(macstr), "%02X:%02X:%02X:%02X:%02X:%02X",
                 best->mac[0], best->mac[1], best->mac[2],
                 best->mac[3], best->mac[4], best->mac[5]);
    }
    ESP_LOGI(TAG, "选中 [%s] rssi=%d mac=%s", best->name, best->rssi,
             best->has_mac ? macstr : "n/a");
    debug_dump_mac("选中", &best->addr, best->mac);
    connect_to(&best->addr, best->type, best->mac, best->has_mac, best->name);
}

// ===========================================================================
// 周期 tick (host 任务, 250ms)
// ===========================================================================
static void tick_cb(struct ble_npl_event *ev) {
    uint32_t now = now_ms();
    uint32_t ph_ms = now - s_ph_ms;

    // ---- 命令处理 ----
    uint8_t cmd = atomic_exchange(&s_cmd, 0);
    if (cmd) invalidate_stream();
    if (cmd == 3) {
        s_sync_done = false;
        if (s_conn != BLE_HS_CONN_HANDLE_NONE) {
            ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
            set_phase(PH_RECONN_WAIT);
        } else if (s_ph == PH_CONNECT_WAIT) {
            ble_gap_conn_cancel();
        } else {
            set_phase(PH_RECONN_WAIT);
        }
        goto rearm;
    }
    if (cmd == 2) {   // 清除配对 + 重扫
        erase_paired();
        s_have_last_addr = false;
        s_force_scan = true;
        ESP_LOGI(TAG, "配对记录已清除, 重新扫描");
        if (s_conn != BLE_HS_CONN_HANDLE_NONE) {
            ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
            set_phase(PH_RECONN_WAIT);
        } else if (s_ph == PH_CONNECT_WAIT) {
            ble_gap_conn_cancel(); // completion callback restarts forced scan
        } else {
            start_scan();
        }
        goto rearm;
    }
    if (cmd == 1) {   // 强制重扫
        s_force_scan = true;
        if (s_conn != BLE_HS_CONN_HANDLE_NONE) {
            ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
            set_phase(PH_RECONN_WAIT);
        } else if (s_ph == PH_CONNECT_WAIT) {
            ble_gap_conn_cancel();
        } else if (s_ph == PH_SCAN) {
            start_scan();
        } else if (s_ph == PH_RUN || s_ph == PH_SYNC) {
            start_scan();
        } else if (s_ph != PH_CONNECT_WAIT) {
            start_scan();
        }
        goto rearm;
    }

    // ---- 阶段处理 ----
    switch (s_ph) {
        case PH_SCAN:
            if (now - s_scan_start_ms > 2500) {
                pick_best_and_connect();
            }
            // 扫描看门狗: 超时未发现则重启扫描 (清空候选槽, 兼容魔方中途休眠唤醒)
            if (s_ph == PH_SCAN && now - s_scan_start_ms > 15000) {
                ESP_LOGI(TAG, "扫描 15s 未发现魔方, 重启扫描 (提示: 转动魔方可唤醒广播)");
                start_scan();
            }
            break;

        case PH_CONNECT_WAIT:
        case PH_MTU:
            if (ph_ms > 10000) fail_and_rescan("连接超时");
            if (s_ph == PH_MTU && ph_ms > 1500) start_disc_svc();   // MTU 事件丢失兜底
            break;

        case PH_DISC_SVC:
        case PH_DISC_CHR:
        case PH_DISC_DSC:
        case PH_SUB_WRITE:
            if (ph_ms > 8000) fail_and_rescan("GATT 发现超时");
            break;

        case PH_SYNC:
            switch (s_devtype) {
                case CUBE_DEV_MOYU_MHC:
                    // Absolute state is mandatory; unsupported firmware stays offline.
                    if (!mhc.state_rx && now - s_last_req_ms > 2000 &&
                        s_sync_retries < 3) {
                        s_sync_retries++;
                        mhc_send_state_req();
                    } else if (!mhc.state_rx && ph_ms > 8000) {
                        fail_and_rescan("MHC absolute state unavailable; bridge stays closed");
                    }
                    break;
                case CUBE_DEV_MOYU_MY3:
                    if (!my3.facelet_rx && now - s_last_req_ms > 2000 &&
                        s_sync_retries < 5) {
                        s_sync_retries++;
                        my3_send_req(163);
                    } else if (!my3.facelet_rx && ph_ms > 12000) {
                        fail_and_rescan("MY3 状态同步超时 (MAC 派生密钥可能错误)");
                    }
                    break;
                case CUBE_DEV_QIYI:
                    if (!qy.hello_rx && now - s_last_req_ms > 1000 &&
                        s_sync_retries < 5) {
                        s_sync_retries++;
                        qy_send_hello();
                    } else if (!qy.hello_rx && ph_ms > 8000) {
                        fail_and_rescan("奇艺 hello 无响应");
                    }
                    break;
                default:
                    break;
            }
            break;

        case PH_RUN:
            // MY3 电量周期查询 (5 分钟)
            if (s_devtype == CUBE_DEV_MOYU_MY3 && now - s_last_req_ms > 300000) {
                my3_send_req(164);
            }
            break;

        case PH_RECONN_WAIT:
            if (ph_ms > 1000) {
                if (s_conn != BLE_HS_CONN_HANDLE_NONE) {
                    ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
                    break; // wait for disconnect; never overwrite a live handle
                }
                if (s_have_last_addr && !s_force_scan) {
                    s_force_scan = false;
                    try_direct_connect();
                } else {
                    s_force_scan = false;
                    start_scan();
                }
            }
            break;

        default:
            break;
    }

rearm:
    ble_npl_callout_reset(&s_tick, pdMS_TO_TICKS(250));
}

// ===========================================================================
// host 就绪回调 (gan_gatt on_sync 后调用, host 任务上下文)
// ===========================================================================
static void on_host_ready(void) {
    if (s_tick_inited) {
        invalidate_stream();
        s_conn = BLE_HS_CONN_HANDLE_NONE;
        s_sync_done = false;
        set_phase(PH_RECONN_WAIT);
        ble_npl_callout_reset(&s_tick, pdMS_TO_TICKS(250));
        return;
    }
    s_tick_inited = true;
    ble_npl_callout_init(&s_tick, nimble_port_get_dflt_eventq(), tick_cb, NULL);
    ble_npl_callout_reset(&s_tick, pdMS_TO_TICKS(250));

    ble_addr_t addr;
    cube_dev_type_t t;
    uint8_t mac[6];
    if (load_paired(&addr, &t, mac)) {
        ESP_LOGI(TAG, "发现配对记录, 直连上次魔方 (type=%d)", (int)t);
        s_last_addr = addr;
        s_have_last_addr = true;
        s_devtype = t;
        memcpy(s_cubemac, mac, 6);
        if (t == CUBE_DEV_MOYU_MY3) {
            gan_crypto_init_base(&my3.crypto, MY3_KEY_BASE, MY3_IV_BASE, s_cubemac);
        }
        snprintf(s_cubename, sizeof(s_cubename), "paired");
        set_phase(PH_CONNECT_WAIT);
        int rc = ble_gap_connect(BLE_OWN_ADDR_RANDOM, &addr, 10000, NULL,
                                 gap_event_cb, NULL);
        if (rc != 0) {
            ESP_LOGW(TAG, "直连失败 rc=%d -> 扫描", rc);
            start_scan();
        }
    } else {
        start_scan();
    }
}

// ===========================================================================
// 对外接口
// ===========================================================================
void cube_client_init(void) {
    s_evtq = xQueueCreate(32, sizeof(cube_client_event_t));
    configASSERT(s_evtq);
    gan_gatt_set_ready_callback(on_host_ready);
}

void cube_client_rescan(void) { invalidate_stream(); atomic_store(&s_cmd, 1); }

void cube_client_clear_paired(void) { invalidate_stream(); atomic_store(&s_cmd, 2); }

void cube_client_request_resync(void) { stream_fault("requested resynchronization"); }
uint32_t cube_client_generation(void) { return atomic_load(&s_generation); }
bool cube_client_generation_valid(uint32_t generation) {
    return generation != 0 && !atomic_load(&s_cmd) && !atomic_load(&s_stream_fault) &&
           generation == atomic_load(&s_generation);
}

bool cube_client_poll(cube_client_event_t *ev) {
    if (!s_evtq) return false;
    return xQueueReceive(s_evtq, ev, 0) == pdTRUE;
}

cube_client_state_t cube_client_state(void) { return s_ext_state; }

const char *cube_client_state_name(void) {
    switch (s_ext_state) {
        case CUBE_CLIENT_SCAN: return "SCAN";
        case CUBE_CLIENT_CONNECTING: return "CONNECT";
        case CUBE_CLIENT_SYNCING: return "SYNC";
        case CUBE_CLIENT_RUN: return "RUN";
        case CUBE_CLIENT_RECONNECT: return "RECONNECT";
        default: return "STOPPED";
    }
}

const char *cube_client_device_name(void) { return s_cubename; }

cube_dev_type_t cube_client_device_type(void) { return s_devtype; }
