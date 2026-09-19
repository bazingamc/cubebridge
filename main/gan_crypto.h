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
// gan_crypto: GAN Gen2/3/4 AES-128 加解密 (基于作者私有 Flutter 项目移植)
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t key[16];
    uint8_t iv[16];
} gan_crypto_t;

// 根据 MAC (显示顺序 A0..A5) 派生 v2 密钥/IV
//   ver = 0 -> 普通 GAN 型号 (如 GAN-356-iCarry / GAN12 ui)
//   ver = 1 -> AiCube 前缀型号 (设备名以 AiCube 开头)
//   派生规则: key[i] = (keyBase[i] + macBytes[5-i]) % 255   (MAC 逆序作盐)
void gan_crypto_init(gan_crypto_t* c, int ver, const uint8_t mac[6]);

// 自定义根密钥/IV 派生 (供其他使用同方案的品牌复用, 如魔域 Moyu32)
void gan_crypto_init_base(gan_crypto_t* c, const uint8_t key_base[16],
                           const uint8_t iv_base[16], const uint8_t mac[6]);

// GAN 自定义 "CBC": 仅对 首块 与 末块(若多块) 做 AES-ECB + IV 异或
//   decode = 解密的逆过程, 与作者私有 Flutter 项目实现逐字节一致
void gan_crypto_decode(const gan_crypto_t* c, uint8_t* data, size_t len);
void gan_crypto_encode(const gan_crypto_t* c, uint8_t* data, size_t len);

#ifdef __cplusplus
}
#endif
