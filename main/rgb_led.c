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

// rgb_led.c - 板载 LED 状态指示实现
//   ESP32-S3 : WS2812 (GPIO48) RMT 驱动, 最低亮度短闪
//   ESP32    : GPIO2 单色 LED, 闪烁频率编码状态
// WS2812 会保持最后收到的颜色, 芯片复位并不会使其熄灭, 上电必须发送黑色帧
#include "rgb_led.h"
#include <string.h>
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include "driver/rmt_tx.h"
#define RGB_LED_PIN GPIO_NUM_48   // ESP32-S3 N16R8 常见开发板 (DevKitC-1 / YD-ESP32-S3 等)
#elif defined(CONFIG_IDF_TARGET_ESP32)
#define RGB_LED_PIN GPIO_NUM_2    // ESP32 DevKitC 板载蓝色 LED
#else
#error "Unsupported IDF target (only ESP32 / ESP32S3 supported)"
#endif

static const char *TAG = "led";
static led_pattern_t s_pattern = LED_OFF;
static uint32_t s_last_tx_ms;      // 上次实际发送 LED 帧的时间 (限频 20ms)
static bool s_inited = false;

// 每通道最低非零码值 (1/255); 配合短闪降低平均亮度。
#define RGB_LED_BRIGHTNESS 3

#if defined(CONFIG_IDF_TARGET_ESP32S3)
// ---------------------------------------------------------------------------
// WS2812 实现 (RMT)
// ---------------------------------------------------------------------------
static rmt_channel_handle_t s_ch;
static rmt_encoder_handle_t s_enc;
static uint8_t s_grb[3]; // RMT 异步发送期间必须保持有效且不可修改

static bool ws2812_send(const uint8_t rgb[3]) {
    if (rmt_tx_wait_all_done(s_ch, 100) != ESP_OK) return false;
    // 确保上一帧锁存，包括初始化后立即切换状态的情况。
    esp_rom_delay_us(300);
    uint8_t out[3];
    for (int i = 0; i < 3; i++) {
        out[i] = (uint8_t)(((uint16_t)rgb[i] * RGB_LED_BRIGHTNESS + 127) / 255);
    }
    s_grb[0] = out[1]; s_grb[1] = out[0]; s_grb[2] = out[2]; // GRB
    rmt_transmit_config_t tx = {
        .loop_count = 0,
        .flags.eot_level = 0,   // 发送完保持低电平 (>50us 即为复位码, 锁存颜色)
    };
    esp_err_t err = rmt_transmit(s_ch, s_enc, s_grb, sizeof(s_grb), &tx);
    if (err != ESP_OK) ESP_LOGW(TAG, "WS2812 transmit: %s", esp_err_to_name(err));
    return err == ESP_OK;
}

static void ws2812_init(void) {
    rmt_tx_channel_config_t ch_cfg = {
        .gpio_num = RGB_LED_PIN,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,   // 10MHz: 1 tick = 100ns
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
        .flags.init_level = 0,               // 空闲电平为低
    };
    rmt_symbol_word_t bit0 = { .duration0 = 3, .level0 = 1, .duration1 = 9, .level1 = 0 };
    rmt_symbol_word_t bit1 = { .duration0 = 9, .level0 = 1, .duration1 = 3, .level1 = 0 };
    rmt_bytes_encoder_config_t enc_cfg = {
        .bit0 = bit0, .bit1 = bit1,
        .flags.msb_first = 1, // WS2812 高位先发，否则亮度 1 会被读成 128
    };

    if (rmt_new_tx_channel(&ch_cfg, &s_ch) != ESP_OK ||
        rmt_new_bytes_encoder(&enc_cfg, &s_enc) != ESP_OK) {
        ESP_LOGW(TAG, "WS2812 通道创建失败, LED 不可用");
        return;
    }
    if (rmt_enable(s_ch) != ESP_OK) return;
    const uint8_t black[3] = {0, 0, 0};
    if (!ws2812_send(black) || rmt_tx_wait_all_done(s_ch, 100) != ESP_OK) return;
    s_inited = true;
}

// 计算当前模式颜色 (rgb 0-255)
static void pattern_color(uint32_t now, uint8_t rgb[3]) {
    switch (s_pattern) {
        case LED_OFF:
            rgb[0] = rgb[1] = rgb[2] = 0;
            break;
        case LED_SCAN: {   // 蓝色 1Hz 慢闪
            bool on = (now % 1000) < 40;
            rgb[0] = 0; rgb[1] = 0; rgb[2] = on ? 255 : 0;
            break;
        }
        case LED_CONNECT: {   // 黄色 5Hz 快闪
            bool on = (now % 200) < 40;
            rgb[0] = on ? 255 : 0; rgb[1] = on ? 180 : 0; rgb[2] = 0;
            break;
        }
        case LED_SYNC: {   // 紫色 2Hz
            bool on = (now % 500) < 40;
            rgb[0] = on ? 180 : 0; rgb[1] = 0; rgb[2] = on ? 255 : 0;
            break;
        }
        case LED_RUN: {   // 绿色微光短闪，每 2.4s 亮 40ms
            bool on = (now % 2400) < 40;
            rgb[0] = 0; rgb[1] = on ? 255 : 0; rgb[2] = 0;
            break;
        }
        case LED_RECONNECT: {   // 橙色 3Hz 快闪
            bool on = (now % 333) < 40;
            rgb[0] = on ? 255 : 0; rgb[1] = on ? 80 : 0; rgb[2] = 0;
            break;
        }
        case LED_ERROR: {   // 红色 5Hz 快闪
            bool on = (now % 200) < 40;
            rgb[0] = on ? 255 : 0; rgb[1] = 0; rgb[2] = 0;
            break;
        }
        case LED_UNPAIR: {   // 红色三短闪，每次 40ms，周期 1.8s
            uint32_t t = now % 1800;
            bool on = (t < 40) || (t >= 200 && t < 240) || (t >= 400 && t < 440);
            rgb[0] = on ? 255 : 0; rgb[1] = 0; rgb[2] = 0;
            break;
        }
        default:
            rgb[0] = rgb[1] = rgb[2] = 0;
            break;
    }
}

void rgb_led_tick(uint32_t now_ms) {
    if (!s_inited) return;
    if (s_pattern == LED_OFF) return;   // 已熄灭, 无需刷新
    if (now_ms - s_last_tx_ms < 20) return;
    s_last_tx_ms = now_ms;
    uint8_t rgb[3];
    pattern_color(now_ms, rgb);
    ws2812_send(rgb);
}

void rgb_led_init(void) {
    ws2812_init();
}

#else
// ---------------------------------------------------------------------------
// ESP32 单色 LED 实现 (GPIO2, 闪烁频率编码)
// ---------------------------------------------------------------------------
static void pattern_level(uint32_t now, bool *level, bool *static_level) {
    *static_level = false;
    switch (s_pattern) {
        case LED_OFF:      *level = false; *static_level = true; break;
        case LED_SCAN:     *level = (now % 1000) < 500; break;   // 1Hz
        case LED_CONNECT:  *level = (now % 200) < 100;  break;   // 5Hz
        case LED_SYNC:     *level = (now % 500) < 250;  break;   // 2Hz
        case LED_RUN:      *level = true; *static_level = true; break;  // 常亮
        case LED_RECONNECT:*level = (now % 333) < 166;  break;   // 3Hz
        case LED_ERROR:    *level = (now % 200) < 100;  break;   // 5Hz
        case LED_UNPAIR: {   // 三短闪
            uint32_t t = now % 1800;
            *level = (t < 100) || (t >= 200 && t < 300) || (t >= 400 && t < 500);
            break;
        }
        default:           *level = false; *static_level = true; break;
    }
}

static void gpio_led_init(void) {
    gpio_config_t io = {0};
    io.pin_bit_mask = 1ULL << RGB_LED_PIN;
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);
    gpio_set_level(RGB_LED_PIN, 0);
    s_inited = true;
}

void rgb_led_tick(uint32_t now_ms) {
    if (!s_inited) return;
    bool level, static_level;
    pattern_level(now_ms, &level, &static_level);
    static int s_last = -1;
    int cur = level ? 1 : 0;
    if (cur != s_last) {
        s_last = cur;
        gpio_set_level(RGB_LED_PIN, cur);
    }
    (void)static_level;
}

void rgb_led_init(void) {
    gpio_led_init();
}

#endif

void rgb_led_set(led_pattern_t p) {
    if (s_pattern == p) return;
    s_pattern = p;
    ESP_LOGI(TAG, "LED -> %d", (int)p);
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    // 立即刷新一次 (熄灭时发黑帧; 其他模式等 tick)
    if (s_inited && p == LED_OFF) {
        const uint8_t black[3] = {0, 0, 0};
        ws2812_send(black);
    }
#else
    if (s_inited) gpio_set_level(RGB_LED_PIN, p == LED_RUN);
#endif
}

led_pattern_t rgb_led_pattern(void) { return s_pattern; }
