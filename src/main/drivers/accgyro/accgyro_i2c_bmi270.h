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

#include "drivers/accgyro/accgyro.h"
#include "drivers/bus.h"

// Gyro output data rate, and so the gyro loop rate: 400, 800 or 1600.
// Choose it so that one six byte read fits comfortably in the period at the
// I2C clock speed in use (about 0.2ms at 400kHz, 0.65ms at 133kHz).
#ifndef BMI270_I2C_GYRO_RATE_HZ
#define BMI270_I2C_GYRO_RATE_HZ 800
#endif

#define BMI270_I2C_ACC_RATE_HZ 200

#define BMI270_I2C_ADDRESS_DEFAULT 0x68

bool bmi270I2cDetect(const extDevice_t *dev);
bool bmi270I2cAccDetect(accDev_t *acc);
bool bmi270I2cGyroDetect(gyroDev_t *gyro);
