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
 * TCA9548A I2C mux with one channel opened at boot and left open.
 *
 * For boards whose flight sensors all sit on one downstream channel of the
 * mux: the channel is selected once, after which the devices behind it are
 * addressed as if they were on the upstream bus. No per-transaction switching.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_I2C) && defined(USE_I2C_MUX_TCA9548)

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_mux_tca9548.h"
#include "drivers/io.h"
#include "drivers/time.h"

#ifndef I2C_MUX_TCA9548_ADDRESS
#define I2C_MUX_TCA9548_ADDRESS 0x70
#endif

static bool muxChannelOpen = false;

bool i2cMuxTca9548Init(void)
{
#ifdef I2C_MUX_TCA9548_RESET_PIN
    // Active low reset: hold it released
    const IO_t resetIO = IOGetByTag(IO_TAG(I2C_MUX_TCA9548_RESET_PIN));
    IOInit(resetIO, OWNER_SYSTEM, 0);
    IOConfigGPIO(resetIO, IOCFG_OUT_PP);
    IOLo(resetIO);
    delay(1);
    IOHi(resetIO);
    delay(1);
#endif

    // The mux has a single control register and no register address: the one
    // data byte is the channel enable mask, sent here in the register position.
    muxChannelOpen = i2cWriteBuffer(I2C_MUX_TCA9548_DEVICE, I2C_MUX_TCA9548_ADDRESS, 1U << I2C_MUX_TCA9548_CHANNEL, 0, NULL);

    return muxChannelOpen;
}

bool i2cMuxTca9548IsOpen(void)
{
    return muxChannelOpen;
}

#endif // USE_I2C && USE_I2C_MUX_TCA9548
