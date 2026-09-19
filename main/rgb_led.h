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

#pragma once
// rgb_led: 板载 LED 状态指示 (双芯片兼容)
//   ESP32-S3 : WS2812 RGB (GPIO48), RMT 驱动, 支持全彩
//   ESP32    : GPIO2 单色 LED, 以闪烁频率区分状态 (无颜色)
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LED_OFF = 0,      // 熄灭
    LED_SCAN,         // 蓝色 慢闪 1Hz     - 正在扫描实体魔方
    LED_CONNECT,      // 黄色 快闪 5Hz     - BLE 连接/发现中
    LED_SYNC,         // 紫色 中速 2Hz     - 已连接, 等待初始状态
    LED_RUN,          // 绿色微光短闪 (ESP32 单色常亮) - 正常转发 (实体魔方在线)
    LED_RECONNECT,    // 橙色 快闪 3Hz     - 掉线重连中
    LED_ERROR,        // 红色 快闪 5Hz     - 错误
    LED_UNPAIR,       // 红色 三短闪      - 清除配对 (约 1.8s 后恢复)
} led_pattern_t;

void rgb_led_init(void);           // 初始化并熄灭 (WS2812 需主动发黑色帧锁存)
void rgb_led_set(led_pattern_t p); // 切换状态模式
led_pattern_t rgb_led_pattern(void);
void rgb_led_tick(uint32_t now_ms); // 主循环周期调用 (约 20ms 更新率)

#ifdef __cplusplus
}
#endif
