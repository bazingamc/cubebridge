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
#include <stdbool.h>
#include <stdint.h>

// BLE random-static address in NimBLE little-endian order. Random part may
// contain neither all zero bits nor all one bits (Bluetooth Core Vol 6 B).
static inline bool gan_address_valid(const uint8_t mac[6]) {
    if ((mac[5] & 0xc0) != 0xc0) return false;
    bool zero = (mac[5] & 0x3f) == 0;
    bool ones = (mac[5] & 0x3f) == 0x3f;
    for (int i = 0; i < 5; i++) {
        zero = zero && mac[i] == 0;
        ones = ones && mac[i] == 0xff;
    }
    return !zero && !ones;
}
