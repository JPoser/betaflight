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

#include "drivers/motor_types.h"

#include "pg/motor.h"

// DShot on the ESP32-S3 dedicated-GPIO bundle (up to eight motors); selected
// in place of the four-channel RMT driver by USE_DSHOT_DEDIC_GPIO.
bool dshotDedicGpioDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig);
