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

// Board config, with USE_CHARGER_BQ25895:
//   CHARGER_BQ25895_I2C_DEVICE         bus the charger is on, e.g. I2CDEV_0
//   CHARGER_BQ25895_INPUT_LIMIT_MA     optional, 100-3250, default 500
//   CHARGER_BQ25895_CHARGE_CURRENT_MA  optional, default 1536
bool bq25895Init(void);
bool bq25895IsConfigured(void);
