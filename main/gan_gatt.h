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
// gan_gatt: BLE 广播身份层 + GATT Server (Gen2) + 命令分发 (ESP-IDF 原生 NimBLE)
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "gan_identity.h"

#ifdef __cplusplus
extern "C" {
#endif

// Per-board random-static identity is stored in NVS ganidentity/address.
// 0 closes the phone session; a valid synchronized source generation enables it.
void gan_gatt_set_bridge_ready(uint32_t generation);
bool gan_gatt_take_fault(void);

// 电量管理
void gan_gatt_set_battery(uint8_t percent);
uint8_t gan_gatt_get_battery(void);

// 是否有客户端订阅了通知特征
bool gan_gatt_is_subscribed(void);

// 排队一个 20 字节明文帧；host 加密并发送。无就绪订阅者时返回 false。
bool gan_gatt_notify(const uint8_t* frame, size_t len);

// 初始化 NVS 身份和 NimBLE；实体完成同步后才允许广播。
void gan_gatt_init(void);

// 设置 host 就绪回调 (host 与 controller 同步完成后调用一次, 运行于 NimBLE host 任务)
// 供 Central 侧 (cube_client) 在此启动扫描/直连; 必须在 gan_gatt_init 之前设置
void gan_gatt_set_ready_callback(void (*cb)(void));

// Main-task command service; BLE transmission/retry runs on the host task.
void gan_gatt_service(void);

#ifdef __cplusplus
}
#endif
