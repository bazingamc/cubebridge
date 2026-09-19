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

// cube_state.cpp - 魔方状态引擎实现 (基于作者私有 Flutter 项目移植)
//   - ca/ea 布局与乘法: edgeMult/cornMult/cubeMult
//   - 移动表:           moveCube
//   - facelet 映射:     cFacelet/eFacelet/toFaceCube
#include "cube_state.h"
#include <stdbool.h>
#include <string.h>

// ---------- 基本转动表 (仅 6 面, GAN v2 事件无法表达中层 M/E/S) ----------
// 与 dart moveCube 基础项 (索引 0,3,6,9,12,15) 完全一致
static const uint8_t BASE_CA[6][8] = {
    {3, 0, 1, 2, 4, 5, 6, 7},          // U
    {20, 1, 2, 8, 15, 5, 6, 19},       // R
    {9, 21, 2, 3, 16, 12, 6, 7},       // F
    {0, 1, 2, 3, 5, 6, 7, 4},          // D
    {0, 10, 22, 3, 4, 17, 13, 7},      // L
    {0, 1, 11, 23, 4, 5, 18, 14},      // B
};
static const uint8_t BASE_EA[6][12] = {
    {6, 0, 2, 4, 8, 10, 12, 14, 16, 18, 20, 22},   // U
    {16, 2, 4, 6, 22, 10, 12, 14, 8, 18, 20, 0},   // R
    {0, 19, 4, 6, 8, 17, 12, 14, 3, 11, 20, 22},   // F
    {0, 2, 4, 6, 10, 12, 14, 8, 16, 18, 20, 22},   // D
    {0, 2, 20, 6, 8, 10, 18, 14, 16, 4, 12, 22},   // L
    {0, 2, 4, 23, 8, 10, 12, 21, 16, 18, 7, 15},   // B
};

// 预计算的 18 个转动: index = face*3 + turn (0=CW 1=180 2=CCW), 与 dart 布局一致
static cube_state_t g_moveCube[18];
static bool g_movesReady = false;

static void corn_mult(const cube_state_t* a, const cube_state_t* b, cube_state_t* prod) {
    for (int cc = 0; cc < 8; cc++) {
        int ori = ((a->ca[b->ca[cc] & 7] >> 3) + (b->ca[cc] >> 3)) % 3;
        prod->ca[cc] = (a->ca[b->ca[cc] & 7] & 7) | (ori << 3);
    }
}

static void edge_mult(const cube_state_t* a, const cube_state_t* b, cube_state_t* prod) {
    for (int ed = 0; ed < 12; ed++) {
        prod->ea[ed] = a->ea[b->ea[ed] >> 1] ^ (b->ea[ed] & 1);
    }
}

static void cube_mult(const cube_state_t* a, const cube_state_t* b, cube_state_t* prod) {
    corn_mult(a, b, prod);
    edge_mult(a, b, prod);
}

static void init_moves(void) {
    if (g_movesReady) return;
    for (int f = 0; f < 6; f++) {
        cube_state_t base;
        memcpy(base.ca, BASE_CA[f], 8);
        memcpy(base.ea, BASE_EA[f], 12);
        g_moveCube[f * 3 + 0] = base;                          // CW
        cube_mult(&base, &base, &g_moveCube[f * 3 + 1]);       // 180 = base*base
        cube_mult(&g_moveCube[f * 3 + 1], &base, &g_moveCube[f * 3 + 2]); // CCW = 180*base
    }
    g_movesReady = true;
}

void cube_init(cube_state_t* c) {
    for (int i = 0; i < 8; i++) c->ca[i] = (uint8_t)i;
    for (int i = 0; i < 12; i++) c->ea[i] = (uint8_t)(i * 2);
}

void cube_move(cube_state_t* c, int face, int turn) {
    init_moves();
    if (face < 0 || face > 5 || turn < 0 || turn > 2) return;
    cube_state_t tmp;
    cube_mult(&g_moveCube[face * 3 + turn], c, &tmp);
    *c = tmp;
}

// facelet 映射表 (csTimer 标准)
static const int8_t C_FACELET[8][3] = {
    {8, 9, 20}, {6, 18, 38}, {0, 36, 47}, {2, 45, 11},
    {29, 26, 15}, {27, 44, 24}, {33, 53, 42}, {35, 17, 51},
};
static const int8_t E_FACELET[12][2] = {
    {5, 10}, {7, 19}, {3, 37}, {1, 46},
    {32, 16}, {28, 25}, {30, 43}, {34, 52},
    {23, 12}, {21, 41}, {50, 39}, {48, 14},
};

void cube_to_facelet(const cube_state_t* c, char out[55]) {
    int perm[54];
    for (int i = 0; i < 54; i++) perm[i] = i;

    for (int cc = 0; cc < 8; cc++) {
        int j = c->ca[cc] & 0x7;
        int ori = c->ca[cc] >> 3;
        for (int n = 0; n < 3; n++) {
            perm[C_FACELET[cc][(n + ori) % 3]] = C_FACELET[j][n];
        }
    }
    for (int ed = 0; ed < 12; ed++) {
        int j = c->ea[ed] >> 1;
        int ori = c->ea[ed] & 1;
        for (int n = 0; n < 2; n++) {
            perm[E_FACELET[ed][(n + ori) % 2]] = E_FACELET[j][n];
        }
    }

    static const char ts[] = "URFDLB";
    for (int i = 0; i < 54; i++) out[i] = ts[perm[i] / 9];
    out[54] = 0;
}

// 由 facelet 字符串反解 cubie (基于作者私有 Flutter 项目移植)
bool cube_from_facelet(cube_state_t* c, const char facelet[55]) {
    // 1) 中心块颜色 -> 面索引
    int8_t f[54];
    const char centers[6] = {facelet[4], facelet[13], facelet[22],
                             facelet[31], facelet[40], facelet[49]};
    uint8_t counts[6] = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < i; j++) {
            if (centers[j] == centers[i]) return false;   // 中心颜色重复
        }
    }
    for (int i = 0; i < 54; i++) {
        int v = -1;
        for (int j = 0; j < 6; j++) {
            if (facelet[i] == centers[j]) { v = j; break; }
        }
        if (v < 0) return false;                          // 非法颜色字符
        f[i] = (int8_t)v;
        counts[v]++;
    }
    for (int i = 0; i < 6; i++) if (counts[i] != 9) return false;

    // 2) 角块: 找朝向 (U/D 色所在位), 再按另两色定位
    uint8_t ca[8];
    for (int i = 0; i < 8; i++) {
        int ori;
        for (ori = 0; ori < 3; ori++) {
            int cv = f[C_FACELET[i][ori]];
            if (cv == 0 || cv == 3) break;
        }
        if (ori == 3) return false;
        int col1 = f[C_FACELET[i][(ori + 1) % 3]];
        int col2 = f[C_FACELET[i][(ori + 2) % 3]];
        ca[i] = 0xff;
        for (int j = 0; j < 8; j++) {
            if (col1 == C_FACELET[j][1] / 9 && col2 == C_FACELET[j][2] / 9) {
                ca[i] = (uint8_t)(j | ((ori % 3) << 3));
                break;
            }
        }
        if (ca[i] == 0xff) return false;                  // 角块匹配失败
    }

    // 3) 棱块
    uint8_t ea[12];
    for (int i = 0; i < 12; i++) {
        ea[i] = 0xff;
        for (int j = 0; j < 12; j++) {
            if (f[E_FACELET[i][0]] == E_FACELET[j][0] / 9 &&
                f[E_FACELET[i][1]] == E_FACELET[j][1] / 9) {
                ea[i] = (uint8_t)(j << 1);
                break;
            }
            if (f[E_FACELET[i][0]] == E_FACELET[j][1] / 9 &&
                f[E_FACELET[i][1]] == E_FACELET[j][0] / 9) {
                ea[i] = (uint8_t)((j << 1) | 1);
                break;
            }
        }
        if (ea[i] == 0xff) return false;                  // 棱块匹配失败
    }

    unsigned corners = 0, edges = 0, twists = 0, flips = 0, cp = 0, ep = 0;
    for (int i = 0; i < 8; i++) {
        unsigned piece = ca[i] & 7;
        if (corners & (1u << piece)) return false;
        corners |= 1u << piece;
        twists += ca[i] >> 3;
        for (int j = 0; j < i; j++) cp ^= (ca[j] & 7) > piece;
    }
    for (int i = 0; i < 12; i++) {
        unsigned piece = ea[i] >> 1;
        if (edges & (1u << piece)) return false;
        edges |= 1u << piece;
        flips ^= ea[i] & 1;
        for (int j = 0; j < i; j++) ep ^= (ea[j] >> 1) > piece;
    }
    if (twists % 3 || flips || cp != ep) return false;
    cube_state_t decoded;
    memcpy(decoded.ca, ca, sizeof(ca));
    memcpy(decoded.ea, ea, sizeof(ea));
    char canonical[55];
    cube_to_facelet(&decoded, canonical);
    for (int i = 0; i < 54; i++) if (canonical[i] != "URFDLB"[f[i]]) return false;

    memcpy(c->ca, ca, 8);
    memcpy(c->ea, ea, 12);
    return true;
}

// ---------- 虚拟魔方运行时 ----------

void vcube_init(vcube_t* v) {
    cube_init(&v->cubie);
    v->serial = 0;
    v->ring_len = 0;
    v->last_ms = 0;
    v->time_valid = false;
    memset(v->ring, 0, sizeof(v->ring));
}

static void vcube_push(vcube_t* v, const vcube_move_t* m, uint32_t now_ms) {
    // Unsigned subtraction preserves wrap; simultaneous source moves stay 0ms.
    uint32_t dt = v->time_valid ? now_ms - v->last_ms : 0;
    if (dt > UINT16_MAX) dt = UINT16_MAX;
    v->last_ms = now_ms;
    v->time_valid = true;

    for (int i = 6; i > 0; i--) v->ring[i] = v->ring[i - 1];
    v->ring[0].face = m->face;
    v->ring[0].power = m->power;
    v->ring[0].timeoff = (uint16_t)dt;
    if (v->ring_len < 7) v->ring_len++;
    v->serial = (uint8_t)(v->serial + 1);
}

int vcube_do_move(vcube_t* v, int face, int turn, uint32_t now_ms, vcube_move_t out[VCUBE_MAX_EVENTS]) {
    if (face < 0 || face > 5 || turn < 0 || turn > 2) return 0;
    int n;

    if (turn == 1) {
        // 180 度: 状态上应用一次 180, 事件层面拆成两次 90 度 CW (GAN v2 事件无 180 编码)
        cube_move(&v->cubie, face, 1);
        out[0].face = (uint8_t)face; out[0].power = 0;
        out[1].face = (uint8_t)face; out[1].power = 0;
        n = 2;
    } else {
        cube_move(&v->cubie, face, turn);  // turn: 0=CW 2=CCW
        out[0].face = (uint8_t)face;
        out[0].power = (turn == 2) ? 1 : 0;
        n = 1;
    }

    for (int i = 0; i < n; i++) vcube_push(v, &out[i], now_ms);
    return n;
}
