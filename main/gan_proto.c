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

// gan_proto.cpp - GAN Gen2 消息编解码实现
// 位流约定与作者私有 Flutter 项目实现一致: 帧内大端, 字节内 MSB 在前
//   bit 0 = byte0 的 bit7, bit 7 = byte0 的 bit0, bit 8 = byte1 的 bit7 ...
#include "gan_proto.h"
#include "gan_identity.h"
#include <string.h>

_Static_assert(sizeof(GAN_HARDWARE_NAME) == 9, "Gen2 hardware name must be 8 bytes");
_Static_assert(GAN_HW_MAJOR >= 0 && GAN_HW_MAJOR <= 255 &&
               GAN_HW_MINOR >= 0 && GAN_HW_MINOR <= 255 &&
               GAN_FW_MAJOR >= 0 && GAN_FW_MAJOR <= 255 &&
               GAN_FW_MINOR >= 0 && GAN_FW_MINOR <= 255,
               "Gen2 version components must fit in one byte");

// 在 MSB-first 位流中写入 n 位 (值 v 的最高位对齐到 start)
static void set_bits(uint8_t* f, int start, int n, uint32_t v) {
    for (int k = 0; k < n; k++) {
        int bp = start + k;
        if ((v >> (n - 1 - k)) & 1u) f[bp >> 3] |= (uint8_t)(0x80 >> (bp & 7));
        else                          f[bp >> 3] &= (uint8_t)~(0x80 >> (bp & 7));
    }
}

int gan_proto_dispatch(const uint8_t* req, size_t len) {
    if (req == NULL || len != GAN_FRAME_LEN) return PROTO_CMD_UNKNOWN;
    switch (req[0]) {
        case 0x04: return PROTO_CMD_FACELETS;  // 请求面贴状态
        case 0x05: return PROTO_CMD_HARDWARE;  // 请求硬件信息
        case 0x09: return PROTO_CMD_BATTERY;   // 请求电量
        case 0x0A: return PROTO_CMD_RESET;       // REQUEST_RESET: 重置为复原态
        default:   return PROTO_CMD_UNKNOWN;
    }
}

// FACELETS 事件 (mode 4): 发送角块0-6与棱块0-10, 角块7/棱块11由主机奇偶校验反推
void gan_proto_facelets(uint8_t frame[GAN_FRAME_LEN], uint8_t serial, const cube_state_t* cubie) {
    memset(frame, 0, GAN_FRAME_LEN);
    set_bits(frame, 0, 4, 4);
    set_bits(frame, 4, 8, serial);   // serial 为 8 位字段 bits[4:12]
    for (int i = 0; i < 7; i++) {
        set_bits(frame, 12 + i * 3, 3, cubie->ca[i] & 7);    // 角块 perm
        set_bits(frame, 33 + i * 2, 2, cubie->ca[i] >> 3);   // 角块 ori
    }
    for (int i = 0; i < 11; i++) {
        set_bits(frame, 47 + i * 4, 4, cubie->ea[i] >> 1);   // 棱块 perm
        set_bits(frame, 91 + i,     1, cubie->ea[i] & 1);    // 棱块 ori
    }
}

// MOVE 事件 (mode 2): 7 个有效步槽, slot0=最新; 历史不足时用最新步重复填充
//   不能使用空槽哨兵 (m>=12), 否则主机 keyChkInc!=0 会跳过计时更新
void gan_proto_move(uint8_t frame[GAN_FRAME_LEN], const vcube_t* v) {
    memset(frame, 0, GAN_FRAME_LEN);
    set_bits(frame, 0, 4, 2);
    set_bits(frame, 4, 8, v->serial);   // serial 为 8 位字段 bits[4:12]
    for (int i = 0; i < 7; i++) {
        int idx = (i < v->ring_len) ? i : 0;   // 历史不足时重复最新步
        uint8_t face = v->ring[idx].face;
        uint8_t power = v->ring[idx].power;
        set_bits(frame, 12 + i * 5, 5, (uint32_t)((face << 1) | power));
        set_bits(frame, 47 + i * 16, 16, v->ring[idx].timeoff);
    }
}

// BATTERY 事件 (mode 9): 电量百分比
int gan_proto_apply_move(uint8_t frames[VCUBE_MAX_EVENTS][GAN_FRAME_LEN], vcube_t *v,
                         int face, int turn, uint32_t timestamp_ms) {
    if (face < 0 || face > 5 || turn < 0 || turn > 2) return 0;
    int count = turn == 1 ? 2 : 1;
    for (int i = 0; i < count; i++) {
        vcube_move_t event[VCUBE_MAX_EVENTS];
        vcube_do_move(v, face, turn == 1 ? 0 : turn, timestamp_ms, event);
        gan_proto_move(frames[i], v);
    }
    return count;
}

void gan_proto_battery(uint8_t frame[GAN_FRAME_LEN], uint8_t percent) {
    memset(frame, 0, GAN_FRAME_LEN);
    set_bits(frame, 0, 4, 9);
    set_bits(frame, 8, 8, percent);
}

// HARDWARE 事件 (mode 5): 按 csTimer gancube.js parseV2Data 的布局。
// bytes 1..2: 硬件版本; 3..4: 固件版本; 5..12: 8字节型号; bit104: gyro。
// 版本与型号为模拟值，并非真机抓包；魔方星球的身份校验仍需实测。
void gan_proto_hardware(uint8_t frame[GAN_FRAME_LEN]) {
    memset(frame, 0, GAN_FRAME_LEN);
    set_bits(frame, 0, 4, 5);
    frame[1] = GAN_HW_MAJOR; frame[2] = GAN_HW_MINOR;
    frame[3] = GAN_FW_MAJOR; frame[4] = GAN_FW_MINOR;
    memcpy(frame + 5, GAN_HARDWARE_NAME, 8);  // 与标准设备信息共享版本配置
    set_bits(frame, 104, 1, 0);              // 网关不提供陀螺仪上报
}

// GYRO 事件 (mode 1): 恒定四元数 (静止), Q15 格式, 与 V2 布局一致
void gan_proto_gyro(uint8_t frame[GAN_FRAME_LEN]) {
    memset(frame, 0, GAN_FRAME_LEN);
    set_bits(frame, 0, 4, 1);
    set_bits(frame, 4, 16, 0x7FFF);  // w
    set_bits(frame, 20, 16, 0);      // x
    set_bits(frame, 36, 16, 0);      // y
    set_bits(frame, 52, 16, 0);      // z
}
