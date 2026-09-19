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
// gan_proto: GAN Gen2 消息编解码与命令分发
// 位布局基于作者私有 Flutter 项目移植
//   - MOVE 事件:    mode 2, serial bits[4:12], 7 x (5bit move + 16bit timeoff)
//   - FACELETS 事件:mode 4, 角块 7x(3bit perm+2bit ori), 棱块 11x(4bit perm+1bit ori)
//   - BATTERY 事件: mode 9, 电量 bits[8:16]
//   - HARDWARE 事件:mode 5, 版本 bytes[1..4], 型号 bytes[5..12], gyro bit104
#include <stdint.h>
#include <stddef.h>
#include "cube_state.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GAN_FRAME_LEN 20

// 命令分发结果
#define PROTO_CMD_UNKNOWN  0
#define PROTO_CMD_FACELETS 1
#define PROTO_CMD_HARDWARE 2
#define PROTO_CMD_BATTERY  3
#define PROTO_CMD_RESET    4   // 0x0A REQUEST_RESET: 重置魔方状态为复原态
                               // (固定报文 0A 05 39 77 00 00 01 23 45 67 89 AB..., 参考 gan-web-bluetooth)

// 解析已解密的请求帧, 返回命令类型
int gan_proto_dispatch(const uint8_t* req, size_t len);

// 构建 20 字节明文事件帧 (调用方负责加密与通知)
void gan_proto_facelets(uint8_t frame[GAN_FRAME_LEN], uint8_t serial, const cube_state_t* cubie);
void gan_proto_move(uint8_t frame[GAN_FRAME_LEN], const vcube_t* v);
// Capture each quarter-turn before applying the next (two distinct half-turn frames).
int gan_proto_apply_move(uint8_t frames[VCUBE_MAX_EVENTS][GAN_FRAME_LEN], vcube_t *v,
                         int face, int turn, uint32_t timestamp_ms);
void gan_proto_battery(uint8_t frame[GAN_FRAME_LEN], uint8_t percent);
void gan_proto_hardware(uint8_t frame[GAN_FRAME_LEN]);
void gan_proto_gyro(uint8_t frame[GAN_FRAME_LEN]);

#ifdef __cplusplus
}
#endif
