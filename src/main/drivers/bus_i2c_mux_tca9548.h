/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>

// Board config, with USE_I2C_MUX_TCA9548:
//   I2C_MUX_TCA9548_DEVICE     upstream bus, e.g. I2CDEV_0
//   I2C_MUX_TCA9548_CHANNEL    downstream channel to open, 0-7
//   I2C_MUX_TCA9548_ADDRESS    optional, default 0x70
//   I2C_MUX_TCA9548_RESET_PIN  optional, the mux's active low reset
bool i2cMuxTca9548Init(void);
bool i2cMuxTca9548IsOpen(void);
