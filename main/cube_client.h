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
// cube_client: BLE Central 实体魔方客户端
//   扫描并连接 魔域(MHC / WCU_MY3) / 奇艺(QY-QYSC) 智能魔方,
//   解析转动与状态事件, 投递到 FreeRTOS 队列由主循环消费,
//   经 vcube 状态引擎转为 GAN 协议转发给魔方星球。
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 支持的实体魔方类型
typedef enum {
    CUBE_DEV_NONE = 0,
    CUBE_DEV_MOYU_MHC,   // 魔域 MHC/WRM  (服务 0x1000, 明文)
    CUBE_DEV_MOYU_MY3,   // 魔域 WCU_MY3  (服务 0783b03e, AES-CBC)
    CUBE_DEV_QIYI,       // 奇艺 QY-QYSC  (服务 0xFFF0/0xFFF6, AES-ECB)
} cube_dev_type_t;

// 客户端状态 (对应 LED 指示)
typedef enum {
    CUBE_CLIENT_STOPPED = 0,
    CUBE_CLIENT_SCAN,        // 扫描中
    CUBE_CLIENT_CONNECTING, // 连接/GATT 发现中
    CUBE_CLIENT_SYNCING,    // 已订阅, 等待初始状态
    CUBE_CLIENT_RUN,        // 正常转发
    CUBE_CLIENT_RECONNECT,  // 掉线重连中
} cube_client_state_t;

// 事件类型
#define CUBE_EVT_NONE      0
#define CUBE_EVT_MOVE      1   // face/power 有效
#define CUBE_EVT_STATE     2   // facelet 有效 (初始状态同步 / 校正)
#define CUBE_EVT_BATTERY   3   // battery 有效
#define CUBE_EVT_CONNECTED 4   // 实体魔方已连接 (开始同步)
#define CUBE_EVT_LOST      5   // 实体魔方断线 (开始重连)

typedef struct {
    uint32_t generation;   // Source connection epoch; stale events are discarded.
    uint32_t timestamp_ms; // Extended source clock, never queue-consumption time.
    uint8_t time_source;
    uint8_t type;         // CUBE_EVT_*
    uint8_t face;         // MOVE: 0-5 (URFDLB)
    uint8_t power;        // MOVE: 0=CW 1=CCW
    uint8_t battery;      // BATTERY: 0-100
    char facelet[55];     // STATE: URFDLB 面贴串 (任意配色字母)
} cube_client_event_t;

#define CUBE_TIME_MHC 1     // 65536 ticks/second
#define CUBE_TIME_MY3 2     // Accumulated on-device millisecond intervals
#define CUBE_TIME_QIYI 3    // 1600 ticks/second

uint32_t cube_client_generation(void);
bool cube_client_generation_valid(uint32_t generation);
// Invalidate immediately; host reconnects and obtains a fresh complete state.
void cube_client_request_resync(void);

// 初始化 (创建队列并注册 host 就绪回调; 由 main 在 gan_gatt_init 之前调用)
// host 就绪后自动开始扫描/直连配对魔方
void cube_client_init(void);

// BOOT 短按: 断开当前连接, 清空配对缓存, 强制重新扫描
void cube_client_rescan(void);

// BOOT 长按: 清除 NVS 配对记录 (回到配对模式)
void cube_client_clear_paired(void);

// 主循环消费事件 (非阻塞), 返回 true 时 *ev 有效
bool cube_client_poll(cube_client_event_t *ev);

// 当前状态 / 已连接魔方信息
cube_client_state_t cube_client_state(void);
const char *cube_client_state_name(void);
const char *cube_client_device_name(void);
cube_dev_type_t cube_client_device_type(void);

#ifdef __cplusplus
}
#endif
