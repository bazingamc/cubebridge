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

// Extend a wrapping source clock before conversion; retain sub-ms remainder.
typedef struct {
    uint32_t raw, ms, remainder;
    bool initialized;
} cube_time_t;

static inline bool cube_time_update(cube_time_t *clock, uint32_t raw,
                                    uint32_t hz, uint32_t *ms) {
    if (!hz) return false;
    if (!clock->initialized) {
        clock->raw = raw;
        clock->initialized = true;
    }
    uint32_t delta = raw - clock->raw;
    if (delta >= UINT32_C(0x80000000)) return false;
    uint64_t scaled = (uint64_t)delta * 1000 + clock->remainder;
    clock->ms += (uint32_t)(scaled / hz);
    clock->remainder = (uint32_t)(scaled % hz);
    clock->raw = raw;
    *ms = clock->ms;
    return true;
}
