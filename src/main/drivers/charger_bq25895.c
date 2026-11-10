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

/*
 * TI BQ25895 single-cell charger / power-path manager: one-time setup.
 *
 * For boards where the flight controller is the only host the charger has.
 * Left alone, its I2C watchdog expires and returns every register to the
 * power-on defaults, which include the 5V boost output enabled and a 2A
 * charge current. This writes a fixed configuration once at boot, with the
 * watchdog disabled so that it stays put.
 *
 * Register values other than the two current limits follow the EMF Tildagon
 * badge firmware (drivers/tildagon_power/bq25895/bq25895.c, bq_init).
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_I2C) && defined(USE_CHARGER_BQ25895)

#include "common/maths.h"

#include "drivers/bus_i2c.h"
#include "drivers/charger_bq25895.h"

#define BQ25895_I2C_ADDRESS 0x6A

#ifndef CHARGER_BQ25895_INPUT_LIMIT_MA
#define CHARGER_BQ25895_INPUT_LIMIT_MA 500
#endif

#ifndef CHARGER_BQ25895_CHARGE_CURRENT_MA
#define CHARGER_BQ25895_CHARGE_CURRENT_MA 1536
#endif

typedef enum {
    BQ25895_REG_INPUT_SOURCE = 0x00,    // EN_HIZ, EN_ILIM, IINLIM
    BQ25895_REG_ADC_CONTROL = 0x02,
    BQ25895_REG_SYSTEM_CONTROL = 0x03,
    BQ25895_REG_CHARGE_CURRENT = 0x04,
    BQ25895_REG_PRECHARGE_TERM = 0x05,
    BQ25895_REG_TIMER_CONTROL = 0x07,
    BQ25895_REG_STATUS = 0x0B,
    BQ25895_REG_DEVICE = 0x14,
} bq25895Register_e;

typedef enum {
    BQ25895_VAL_TIMER_CONTROL = 0x8C,   // termination on, watchdog off, 12 hour safety timer
    BQ25895_VAL_ADC_CONTROL = 0x60,     // ADC converting once a second, no input source detection
    BQ25895_VAL_SYSTEM_CONTROL = 0x10,  // boost (OTG) off, charging on, 3.0V minimum system voltage
    BQ25895_VAL_PRECHARGE_TERM = 0x00,  // 64mA precharge, 64mA termination
    BQ25895_VAL_EN_ILIM = 0x40,         // let the ILIM pin resistor cap the input current
    BQ25895_VAL_DEVICE_MASK = 0x38,     // part number field of the device register
    BQ25895_VAL_DEVICE_BQ25895 = 0x38,
} bq25895ConfigValues_e;

// Above this the ILIM pin resistor would be the tighter limit, so it is disabled
#define BQ25895_ILIM_PIN_MAX_MA 1500

static bool chargerConfigured = false;

static bool bq25895Write(bq25895Register_e reg, uint8_t value)
{
    return i2cWrite(CHARGER_BQ25895_I2C_DEVICE, BQ25895_I2C_ADDRESS, reg, value);
}

bool bq25895Init(void)
{
    uint8_t device = 0;

    chargerConfigured = false;

    if (!i2cRead(CHARGER_BQ25895_I2C_DEVICE, BQ25895_I2C_ADDRESS, BQ25895_REG_DEVICE, 1, &device)
        || (device & BQ25895_VAL_DEVICE_MASK) != BQ25895_VAL_DEVICE_BQ25895) {
        return false;
    }

    const unsigned inputLimitMa = constrain(CHARGER_BQ25895_INPUT_LIMIT_MA, 100, 3250);
    const unsigned chargeCurrentMa = constrain(CHARGER_BQ25895_CHARGE_CURRENT_MA, 0, 5056);

    // Input limit: 100mA offset, 50mA per step
    uint8_t inputSource = (inputLimitMa - 100) / 50;
    if (inputLimitMa <= BQ25895_ILIM_PIN_MAX_MA) {
        inputSource |= BQ25895_VAL_EN_ILIM;
    }

    // Watchdog first: once it is off nothing below can be undone by a timeout.
    // Then the boost, which the power-on defaults leave enabled.
    bool ok = bq25895Write(BQ25895_REG_TIMER_CONTROL, BQ25895_VAL_TIMER_CONTROL);
    ok = bq25895Write(BQ25895_REG_SYSTEM_CONTROL, BQ25895_VAL_SYSTEM_CONTROL) && ok;
    ok = bq25895Write(BQ25895_REG_ADC_CONTROL, BQ25895_VAL_ADC_CONTROL) && ok;
    ok = bq25895Write(BQ25895_REG_CHARGE_CURRENT, chargeCurrentMa / 64) && ok;      // 64mA per step
    ok = bq25895Write(BQ25895_REG_PRECHARGE_TERM, BQ25895_VAL_PRECHARGE_TERM) && ok;
    ok = bq25895Write(BQ25895_REG_INPUT_SOURCE, inputSource) && ok;

    chargerConfigured = ok;

    return ok;
}

bool bq25895IsConfigured(void)
{
    return chargerConfigured;
}

#endif // USE_I2C && USE_CHARGER_BQ25895
