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
// cube_state: 魔方状态引擎 (基于作者私有 Flutter 项目移植) + 虚拟魔方运行时
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------- CubieCube 纯模型 (与 csTimer mathlib.js 一致) ----------
// ca[i] = orientation << 3 | permutation   (角块, 8 个)
// ea[i] = permutation << 1 | orientation   (棱块, 12 个)
typedef struct {
    uint8_t ca[8];
    uint8_t ea[12];
} cube_state_t;

void cube_init(cube_state_t* c);                       // 复原态
// 转动: face 0=U 1=R 2=F 3=D 4=L 5=B; turn 0=CW 1=180 2=CCW
void cube_move(cube_state_t* c, int face, int turn);
// 输出 54 字符 facelet 字符串 (URFDLB 顺序), 用于调试显示
void cube_to_facelet(const cube_state_t* c, char out[55]);
// 由 54 字符 facelet 字符串 (URFDLB 顺序, 任意配色) 反解 cubie 状态
// 基于作者私有 Flutter 项目移植; 中心块不合法时返回 false 且不修改 c
bool cube_from_facelet(cube_state_t* c, const char facelet[55]);

// ---------- 虚拟魔方运行时 (serial + 最近7步缓冲) ----------
// GAN v2 MOVE 事件固定携带 7 个有效步槽 (不能有空槽, 否则主机跳过计时更新)
typedef struct {
    uint8_t face;    // 0-5
    uint8_t power;   // 0=CW 1=CCW (GAN v2 事件仅 1 bit)
    uint16_t timeoff;// 与上一步的时间间隔 (ms)
} vcube_ring_t;

typedef struct {
    cube_state_t cubie;      // 当前 cubie 状态
    uint8_t serial;          // 步数计数 0-255 循环递增
    uint8_t ring_len;        // 已有历史步数 (0-7)
    vcube_ring_t ring[7];    // ring[0] = 最新
    uint32_t last_ms;        // 上一步的毫秒时间戳
    bool time_valid;
} vcube_t;

// MOVE 事件描述 (VCUBE_MAX_EVENTS=2 用于 180 度拆成两次 90 度)
typedef struct {
    uint8_t face;
    uint8_t power;   // 0=CW 1=CCW
} vcube_move_t;
#define VCUBE_MAX_EVENTS 2

void vcube_init(vcube_t* v);
// 施加一次虚拟转动, 返回产生的事件数 (180 返回 2)
// now_ms 为当前毫秒时间戳 (用于计算 timeoff)
int vcube_do_move(vcube_t* v, int face, int turn, uint32_t now_ms, vcube_move_t out[VCUBE_MAX_EVENTS]);

#ifdef __cplusplus
}
#endif
