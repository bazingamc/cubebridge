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

// gan_crypto.cpp - GAN AES-128 加解密实现 (基于作者私有 Flutter 项目移植)
//   - 根密钥/IV 与密钥派生/加解密器逻辑均来自该私有项目
// 使用 ESP32 硬件 AES (ESP-IDF mbedtls 组件提供的 esp_aes API)
#include "gan_crypto.h"
#include "aes/esp_aes.h"
#include <string.h>

// GAN v2/v3/v4 根密钥与 IV (ver=0, 普通型号)
static const uint8_t KEY_V2[16]    = {1, 2, 66, 40, 49, 145, 22, 7, 32, 5, 24, 84, 66, 17, 18, 83};
static const uint8_t IV_V2[16]     = {17, 3, 50, 40, 33, 1, 118, 39, 32, 149, 120, 20, 50, 18, 2, 67};

// GAN v2/v3/v4 根密钥与 IV (ver=1, AiCube 前缀型号)
static const uint8_t KEY_V2_V1[16] = {5, 18, 2, 69, 2, 1, 41, 86, 18, 120, 18, 118, 129, 1, 8, 3};
static const uint8_t IV_V2_V1[16]  = {1, 68, 40, 6, 134, 33, 34, 40, 81, 5, 8, 49, 130, 2, 33, 6};

void gan_crypto_init_base(gan_crypto_t* c, const uint8_t key_base[16],
                           const uint8_t iv_base[16], const uint8_t mac[6]) {
    // 先整份拷贝根密钥/IV (派生自完整副本, 与作者私有 Flutter 项目一致)
    memcpy(c->key, key_base, 16);
    memcpy(c->iv, iv_base, 16);
    for (int i = 0; i < 6; i++) {   // 仅前 6 字节用 MAC 逆序作盐 (与开源实现一致)
        // 注意: 与开源实现保持一致使用 % 255 (不是 % 256)
        c->key[i] = (uint8_t)((c->key[i] + mac[5 - i]) % 255);
        c->iv[i]  = (uint8_t)((c->iv[i] + mac[5 - i]) % 255);
    }
}

void gan_crypto_init(gan_crypto_t* c, int ver, const uint8_t mac[6]) {
    const uint8_t* kb = (ver == 1) ? KEY_V2_V1 : KEY_V2;
    const uint8_t* ib = (ver == 1) ? IV_V2_V1 : IV_V2;
    gan_crypto_init_base(c, kb, ib, mac);
}

// 单个 16 字节块 AES-128 (通过 ESP32 硬件 AES 加速)
static void aes_ecb(uint8_t out[16], const uint8_t in[16], const uint8_t key[16], bool encrypt) {
    esp_aes_context ctx;
    esp_aes_init(&ctx);
    esp_aes_setkey(&ctx, key, 128);
    esp_aes_crypt_ecb(&ctx, encrypt ? ESP_AES_ENCRYPT : ESP_AES_DECRYPT, in, out);
    esp_aes_free(&ctx);
}

// 解密: 先解末块(多块时), 再解首块; 每块解密后与 IV 异或
// 与 _AesCbcDecoder.decode 逻辑逐字节一致 (20 字节帧首末块重叠属正常现象)
void gan_crypto_decode(const gan_crypto_t* c, uint8_t* ret, size_t len) {
    if (len < 16) return;

    if (len > 16) {
        size_t off = len - 16;
        uint8_t block[16], out[16];
        memcpy(block, ret + off, 16);
        aes_ecb(out, block, c->key, false);
        for (int i = 0; i < 16; i++) ret[off + i] = out[i] ^ c->iv[i];
    }

    uint8_t out[16];
    aes_ecb(out, ret, c->key, false);
    for (int i = 0; i < 16; i++) ret[i] = out[i] ^ c->iv[i];
}

// 加密: 先首块 (XOR IV -> AES), 再末块 (XOR IV -> AES)
// 与 _AesCbcDecoder.encode 逻辑逐字节一致
void gan_crypto_encode(const gan_crypto_t* c, uint8_t* ret, size_t len) {
    if (len < 16) return;

    for (int i = 0; i < 16; i++) ret[i] ^= c->iv[i];
    uint8_t out[16];
    aes_ecb(out, ret, c->key, true);
    memcpy(ret, out, 16);

    if (len > 16) {
        size_t off = len - 16;
        uint8_t block[16];
        memcpy(block, ret + off, 16);
        for (int i = 0; i < 16; i++) block[i] ^= c->iv[i];
        aes_ecb(out, block, c->key, true);
        memcpy(ret + off, out, 16);
    }
}
