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

// main.c - ESP32 实体魔方 -> GAN 虚拟魔方网关 (ESP-IDF)
//
// 数据流:
//   实体魔方(魔域MHC/WCU_MY3/奇艺) --BLE Central--> ESP32
//     --vcube 状态引擎--> GAN Gen2 协议 --BLE Peripheral--> 魔方星球
//
// 功能:
//   - BLE 双角色: Central 连接实体魔方 + Peripheral 被魔方星球连接
//   - 实体魔方转动/状态/电量实时转发 (打乱还原全同步)
//   - BOOT 按键: 短按(<1s) 强制重扫; 长按(≥3s) 清除配对 (红色三短闪确认)
//   - LED 状态机: 蓝慢闪=扫描 黄快闪=连接 紫2Hz=同步 绿呼吸=运行 橙3Hz=重连
//   - UART 只读；CONFIG_VCUBE_TEST_INPUT 才启用本地离线测试
//
// 串口命令 (115200, UART0；转动/scramble/bat 仅开发配置可用，不经 BLE 转发):
//   U R F D L B      单步顺转     U' R2 F' ...   标准记号 (支持 U/R/F/D/L/B)
//   scramble        随机打乱 20 步 (演示用)
//   state / ?       打印当前面贴状态 / serial
//   bat <0-100>     设置电量
//   help            帮助
//
// 板型: 由 idf.py set-target 决定 (esp32 / esp32s3), 引脚用 CONFIG_IDF_TARGET_* 宏自动切换
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "cube_state.h"
#include "gan_gatt.h"
#include "gan_proto.h"
#include "cube_client.h"
#include "rgb_led.h"

vcube_t g_vcube;
static const char *TAG = "gan";

#ifdef CONFIG_VCUBE_TEST_INPUT
// 按键引脚: 不同芯片可用 GPIO 不同, 按 IDF 目标宏自动切换
#if defined(CONFIG_IDF_TARGET_ESP32S3)
static const gpio_num_t BTN_PINS[6] = {
    GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7, GPIO_NUM_15, GPIO_NUM_16};
#elif defined(CONFIG_IDF_TARGET_ESP32)
static const gpio_num_t BTN_PINS[6] = {
    GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_14};
#else
#error "Unsupported IDF target (only ESP32 / ESP32S3 supported)"
#endif

#endif // CONFIG_VCUBE_TEST_INPUT

// BOOT 按键 (两芯片均为 GPIO0, 板载上拉, 按下为 LOW)
#define BOOT_BTN_PIN     GPIO_NUM_0
#define BOOT_SHORT_MS    1000   // 短按上限
#define BOOT_LONG_MS     3000   // 长按阈值
#define UNPAIR_LED_MS    2100   // 清配对红色三短闪指示时长

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
static void delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

// ---------------------------------------------------------------------------
// 虚拟转动应用: 更新状态 + 发送 MOVE 事件通知
// ---------------------------------------------------------------------------
static bool s_source_synced;
static uint32_t s_source_generation;

static void apply_move(int face, int turn, uint32_t timestamp_ms, bool forward) {
    uint8_t frames[VCUBE_MAX_EVENTS][GAN_FRAME_LEN];
    int n = gan_proto_apply_move(frames, &g_vcube, face, turn, timestamp_ms);
    for (int i = 0; forward && i < n; i++) gan_gatt_notify(frames[i], GAN_FRAME_LEN);
    ESP_LOGD(TAG, "move %c serial=%u", "URFDLB"[face], g_vcube.serial);
}

static void invalidate_source(void) {
    s_source_synced = false;
    gan_gatt_set_bridge_ready(0);
    cube_client_request_resync();
}

static void service_cube_events(void) {
    uint32_t current = cube_client_generation();
    if (current != s_source_generation || !cube_client_generation_valid(current)) {
        s_source_synced = false;
        s_source_generation = current;
        gan_gatt_set_bridge_ready(0);
    }
    cube_client_event_t ev;
    while (cube_client_poll(&ev)) {
        if (!cube_client_generation_valid(ev.generation)) continue;
        if (ev.generation != s_source_generation) {
            s_source_synced = false;
            s_source_generation = ev.generation;
            gan_gatt_set_bridge_ready(0);
        }
        switch (ev.type) {
            case CUBE_EVT_MOVE:
                if (!s_source_synced) { invalidate_source(); break; }
                if (!g_vcube.time_valid && ev.time_source == CUBE_TIME_MY3) {
                    // MY3 supplies a real first interval from a zero accumulator.
                    g_vcube.time_valid = true;
                }
                apply_move(ev.face, ev.power ? 2 : 0, ev.timestamp_ms, true);
                break;
            case CUBE_EVT_STATE: {
                cube_state_t state;
                if (!cube_from_facelet(&state, ev.facelet)) { invalidate_source(); break; }
                if (s_source_synced) {
                    if (memcmp(&state, &g_vcube.cubie, sizeof(state)) != 0) invalidate_source();
                    break; // Identical snapshots must not erase timing/history.
                }
                vcube_init(&g_vcube);
                g_vcube.cubie = state;
                gan_gatt_set_battery(0); // unknown until source reports it
                s_source_synced = true;
                ESP_LOGI(TAG, "Source synchronized generation=%lu", (unsigned long)ev.generation);
                break;
            }
            case CUBE_EVT_BATTERY: {
                gan_gatt_set_battery(ev.battery);
                uint8_t frame[GAN_FRAME_LEN];
                gan_proto_battery(frame, gan_gatt_get_battery());
                gan_gatt_notify(frame, sizeof(frame));
                break;
            }
            case CUBE_EVT_LOST: invalidate_source(); break;
            default: break;
        }
    }
    if (s_source_synced && cube_client_state() == CUBE_CLIENT_RUN &&
        cube_client_generation_valid(s_source_generation)) {
        gan_gatt_set_bridge_ready(s_source_generation);
    }
}

// ---------------------------------------------------------------------------
// 串口命令处理
// ---------------------------------------------------------------------------
static char s_line[64];
static size_t s_line_len = 0;

static void cmd_help(void) {
    ESP_LOGI(TAG, "Commands: state | ? | help");
#ifdef CONFIG_VCUBE_TEST_INPUT
    ESP_LOGW(TAG, "Local test mode: moves/scramble/bat are offline only, never forwarded");
#endif
}

#ifdef CONFIG_VCUBE_TEST_INPUT
static bool test_input_allowed(void) {
    cube_client_state_t state = cube_client_state();
    return !s_source_synced && !gan_gatt_is_subscribed() &&
        (state == CUBE_CLIENT_STOPPED || state == CUBE_CLIENT_SCAN);
}
static void cmd_scramble(void) {
    for (int i = 0; i < 20; i++) {
        int face = (int)(esp_random() % 6);
        int turn = (int)(esp_random() % 3);
        if (!test_input_allowed()) break;
        apply_move(face, turn, now_ms(), false);
        delay_ms(250);
    }
    ESP_LOGI(TAG, "[cmd] 打乱完成");
}

#endif

static void process_line(const char *line) {
    // 去首尾空白
    const char *p = line;
    while (*p == ' ' || *p == '\t') p++;
    size_t len = strlen(p);
    while (len > 0 && (p[len - 1] == ' ' || p[len - 1] == '\t' || p[len - 1] == '\r' || p[len - 1] == '\n')) len--;

    if (len == 0) return;

    if (len == 4 && strncasecmp(p, "help", 4) == 0) { cmd_help(); return; }
    if (len == 5 && strncasecmp(p, "state", 5) == 0) {
        char fl[55];
        cube_to_facelet(&g_vcube.cubie, fl);
        ESP_LOGI(TAG, "[state] serial=%d len=%d facelet=%s", g_vcube.serial, g_vcube.ring_len, fl);
        return;
    }
    if (len == 1 && p[0] == '?') {
        char fl[55];
        cube_to_facelet(&g_vcube.cubie, fl);
        ESP_LOGI(TAG, "[state] serial=%d len=%d facelet=%s", g_vcube.serial, g_vcube.ring_len, fl);
        return;
    }
#ifdef CONFIG_VCUBE_TEST_INPUT
    if (!test_input_allowed()) {
        ESP_LOGW(TAG, "Local test input disabled while source is connected");
        return;
    }
    if (len == 8 && strncasecmp(p, "scramble", 8) == 0) { cmd_scramble(); return; }
    if (len >= 3 && strncasecmp(p, "bat", 3) == 0) {
        int v = atoi(p + 3);
        if (v < 0) v = 0;
        if (v > 100) v = 100;
        gan_gatt_set_battery((uint8_t)v);
        ESP_LOGI(TAG, "[cmd] 电量=%u%%", gan_gatt_get_battery());
        return;
    }

    // 标准记号: U R F D L B + 可选 ' 或 2
    char c = (char)toupper((unsigned char)p[0]);
    int face = -1;
    const char *faces = "URFDLB";
    for (int i = 0; i < 6; i++) {
        if (faces[i] == c) { face = i; break; }
    }
    if (face < 0 || len > 2) {
        ESP_LOGW(TAG, "[cmd] 无法解析: %.*s", (int)len, p);
        return;
    }
    int turn = 0;
    if (len > 1) {
        if (p[1] == '\'') turn = 2;
        else if (p[1] == '2') turn = 1;
    }
    apply_move(face, turn, now_ms(), false);
#else
    ESP_LOGW(TAG, "Bridge accepts read-only commands only (enable CONFIG_VCUBE_TEST_INPUT for local test input)");
#endif
}

static void poll_serial(void) {
    uint8_t ch;
    while (uart_read_bytes(UART_NUM_0, &ch, 1, 0) == 1) {
        if (ch == '\n' || ch == '\r') {
            if (s_line_len > 0) {
                s_line[s_line_len] = '\0';
                process_line(s_line);
                s_line_len = 0;
            }
        } else if (s_line_len < sizeof(s_line) - 1) {
            s_line[s_line_len++] = (char)ch;
        }
    }
}

// ---------------------------------------------------------------------------
// 6 键虚拟输入 (上拉, 按下为 LOW)
// 实体魔方在线 (同步/运行) 时禁用, 避免双状态源失步
// ---------------------------------------------------------------------------
#ifdef CONFIG_VCUBE_TEST_INPUT
static bool s_btnPrev[6] = {};
static uint32_t s_btnLast[6] = {};

static void poll_buttons(void) {
    if (!test_input_allowed()) {
        for (int i = 0; i < 6; i++) s_btnPrev[i] = false;
        return;
    }
    for (int i = 0; i < 6; i++) {
        bool pressed = (gpio_get_level(BTN_PINS[i]) == 0);
        if (pressed != s_btnPrev[i]) {
            s_btnLast[i] = now_ms();
            s_btnPrev[i] = pressed;
        } else if (pressed && (now_ms() - s_btnLast[i]) > 50) {
            s_btnLast[i] = now_ms();
            apply_move(i, 0, now_ms(), false);   // 按键 = 顺转
        }
    }
}

// ---------------------------------------------------------------------------
#endif // CONFIG_VCUBE_TEST_INPUT

// BOOT 按键: 短按(<1s) 强制重扫; 长按(≥3s) 清除配对 (LED_UNPAIR 三短闪确认)
// ---------------------------------------------------------------------------
static bool s_boot_prev;
static uint32_t s_boot_down_ms;
static bool s_boot_long_fired;
static uint32_t s_led_override_until;   // LED 临时覆盖截止时间 (清配对确认)

static void poll_boot_button(void) {
    bool pressed = (gpio_get_level(BOOT_BTN_PIN) == 0);
    uint32_t now = now_ms();

    if (pressed && !s_boot_prev) {
        s_boot_down_ms = now;
        s_boot_long_fired = false;
    } else if (pressed && s_boot_prev && !s_boot_long_fired &&
               now - s_boot_down_ms >= BOOT_LONG_MS) {
        // 长按: 按住满 3s 即触发 (无需等待释放)
        s_boot_long_fired = true;
        cube_client_clear_paired();
        rgb_led_set(LED_UNPAIR);
        s_led_override_until = now + UNPAIR_LED_MS;
        ESP_LOGI(TAG, "[boot] 长按: 清除配对记录, 回到配对模式");
    } else if (!pressed && s_boot_prev && !s_boot_long_fired &&
               now - s_boot_down_ms >= 50 && now - s_boot_down_ms < BOOT_SHORT_MS) {
        // 短按: 释放时触发 (≥50ms 消抖)
        cube_client_rescan();
        ESP_LOGI(TAG, "[boot] 短按: 强制重新扫描");
    }
    s_boot_prev = pressed;
}

// ---------------------------------------------------------------------------
// LED 状态机: cube_client 状态 -> LED 模式 (rgb_led 兼容 S3 WS2812 / ESP32 单色)
// ---------------------------------------------------------------------------
static void led_update(void) {
    uint32_t now = now_ms();
    if (s_led_override_until != 0) {
        if (now < s_led_override_until) {
            rgb_led_tick(now);   // 覆盖期间保持指示 (如清配对三短闪)
            return;
        }
        s_led_override_until = 0;
    }
    switch (cube_client_state()) {
        case CUBE_CLIENT_CONNECTING: rgb_led_set(LED_CONNECT);   break;
        case CUBE_CLIENT_SYNCING:    rgb_led_set(LED_SYNC);      break;
        case CUBE_CLIENT_RUN:        rgb_led_set(LED_RUN);       break;
        case CUBE_CLIENT_RECONNECT:  rgb_led_set(LED_RECONNECT); break;
        default:                     rgb_led_set(LED_SCAN);      break;  // SCAN/STOPPED
    }
    rgb_led_tick(now);
}

// ---------------------------------------------------------------------------
// 主逻辑
// ---------------------------------------------------------------------------
void app_main(void) {
    // 0) 板载 LED (S3: WS2812 发黑帧锁存熄灭 / ESP32: GPIO2 置低)
    rgb_led_init();

    // 1) UART0 (命令输入 / 日志)
    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_cfg);

    // 2) 按键 GPIO (6 个虚拟转动键 + BOOT 键, 均上拉输入)
    gpio_config_t io = {0};
    uint64_t mask = 1ULL << BOOT_BTN_PIN;
#ifdef CONFIG_VCUBE_TEST_INPUT
    for (int i = 0; i < 6; i++) mask |= (1ULL << BTN_PINS[i]);
#endif
    io.pin_bit_mask = mask;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);

    vcube_init(&g_vcube);
    gan_gatt_set_battery(0);

    // 3) 先注册 host 就绪回调, 再启动 NimBLE (双角色: Central + Peripheral)
    cube_client_init();
    gan_gatt_init();

    ESP_LOGI(TAG, "=== ESP32 实体魔方 -> GAN 虚拟魔方网关 ===");
    ESP_LOGI(TAG, "BOOT 短按=重扫 长按(3s)=清配对 | 支持: 魔域MHC/WCU_MY3 奇艺QY-QYSC");
    cmd_help();

    while (1) {
        if (gan_gatt_take_fault()) invalidate_source();
        // Snapshot commands run before source moves; both own g_vcube here.
        gan_gatt_service();
        if (gan_gatt_take_fault()) invalidate_source();
        service_cube_events();
        poll_boot_button();
#ifdef CONFIG_VCUBE_TEST_INPUT
        poll_buttons();
#endif
        poll_serial();
        led_update();
        vTaskDelay(pdMS_TO_TICKS(10));   // >=1 tick (100Hz), 否则 vTaskDelay(0) 不释放 CPU 会饿死 IDLE0 触发看门狗
    }
}
