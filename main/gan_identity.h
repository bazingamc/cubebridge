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
#include <stddef.h>
#include <stdint.h>

// 虚拟 GAN 的统一身份配置，不代表桥接的实体魔方或 ESP-IDF 版本。
// Gen2 内部型号只有 8 字节；广播名/标准 Model Number 使用完整名称。
// 这些值是模拟配置，魔方星球是否要求真机型号代码仍需抓包确认。
#define GAN_DEVICE_NAME       "GAN356iCarry"
#define GAN_HARDWARE_NAME     "GAN356iC"
#define GAN_MANUFACTURER      "GANCUBE"
#define GAN_HW_MAJOR          1
#define GAN_HW_MINOR          1
#define GAN_FW_MAJOR          1
#define GAN_FW_MINOR          0

#define GAN_STRINGIFY_IMPL(x) #x
#define GAN_STRINGIFY(x) GAN_STRINGIFY_IMPL(x)
#define GAN_HARDWARE_VERSION GAN_STRINGIFY(GAN_HW_MAJOR) "." GAN_STRINGIFY(GAN_HW_MINOR)
#define GAN_FIRMWARE_VERSION GAN_STRINGIFY(GAN_FW_MAJOR) "." GAN_STRINGIFY(GAN_FW_MINOR)

// Bluetooth Device Information Service: 可直接读，不依赖 GAN CCCD/AES。
#define GAN_DIS_UUID          0x180A
#define GAN_DIS_MODEL_UUID    0x2A24
#define GAN_DIS_FIRMWARE_UUID 0x2A26
#define GAN_DIS_HARDWARE_UUID 0x2A27
#define GAN_DIS_VENDOR_UUID   0x2A29

static inline const char *gan_identity_value(uint16_t uuid) {
    switch (uuid) {
        case GAN_DIS_MODEL_UUID: return GAN_DEVICE_NAME;
        case GAN_DIS_FIRMWARE_UUID: return GAN_FIRMWARE_VERSION;
        case GAN_DIS_HARDWARE_UUID: return GAN_HARDWARE_VERSION;
        case GAN_DIS_VENDOR_UUID: return GAN_MANUFACTURER;
        default: return NULL;
    }
}
