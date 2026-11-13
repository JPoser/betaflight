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
 * DShot on the ESP32-S3 dedicated-GPIO bundle.
 *
 * The RMT driver (dshot_esp32.c) is limited to the four RMT transmit channels.
 * This driver instead maps up to eight motor pins onto the core-0 dedicated
 * GPIO channels, which the CPU writes together with a single instruction, and
 * bit-bangs the frame against the cycle counter. Every DShot bit is three
 * writes: all pins rise at the bit start, the pins sending a zero fall at
 * 37.5% of the bit period, the pins sending a one fall at 75%.
 *
 * The whole frame is sent with interrupts masked so that no handler can
 * stretch a pulse: 16 bit periods, i.e. about 53us at DShot300. The send
 * routine and everything it touches live in RAM so that a flash cache miss
 * cannot stretch a pulse either.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#if defined(USE_DSHOT) && defined(USE_DSHOT_DEDIC_GPIO)

#include "common/utils.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "hal/dedic_gpio_ll.h"
#include "hal/dedic_gpio_cpu_ll.h"
#include "hal/gpio_ll.h"
#pragma GCC diagnostic pop

#include "soc/gpio_struct.h"
#include "soc/gpio_sig_map.h"
#include "esp_rom_gpio.h"

#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"
#include "drivers/motor_impl.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"

#include "pg/motor.h"

#include "dshot_dedic_gpio_esp32.h"

// Eight dedicated GPIO output channels per core (SOC_DEDIC_GPIO_OUT_CHANNELS_NUM)
#define DSHOT_DEDIC_MAX_MOTORS  8
#define DSHOT_FRAME_BITS        16

typedef struct {
    dshotProtocolControl_t protocolControl;
    IO_t io;
    uint16_t packet;
} dshotDedicMotor_t;

static dshotDedicMotor_t dedicMotors[DSHOT_DEDIC_MAX_MOTORS];
static uint8_t dedicMotorCount = 0;
static bool dedicEnabled = false;

// Frame timing in CPU cycles, measured from the start of the frame. Deliberately
// not const: const data is read from flash through the data cache, and a cache
// miss inside the frame would stretch a pulse.
typedef struct {
    uint32_t bitStart[DSHOT_FRAME_BITS + 1];    // rising edge of each bit; [16] is the frame end
    uint32_t t0h;                               // high time of a zero bit
    uint32_t t1h;                               // high time of a one bit
    uint32_t channelMask;                       // dedicated GPIO channels in use
} dshotDedicTiming_t;

static dshotDedicTiming_t dedicTiming;

__attribute__((always_inline)) static inline uint32_t dedicCycleCount(void)
{
    uint32_t count;
    __asm__ volatile ("rsr.ccount %0" : "=a"(count));
    return count;
}

// ones[bit] is the mask of channels sending a one in that bit, MSB first.
// Must stay self-contained: no calls into flash-resident code.
static FAST_CODE_NOINLINE_CRITICAL void dshotDedicSendFrame(const dshotDedicTiming_t *timing, const uint32_t *ones)
{
    const uint32_t mask = timing->channelMask;
    const uint32_t t0h = timing->t0h;
    const uint32_t t1h = timing->t1h;

    uint32_t savedPs;
    __asm__ volatile ("rsil %0, 15" : "=a"(savedPs));

    const uint32_t start = dedicCycleCount();
    for (unsigned bit = 0; bit < DSHOT_FRAME_BITS; bit++) {
        const uint32_t t0 = timing->bitStart[bit];
        while (dedicCycleCount() - start < t0) { }
        dedic_gpio_cpu_ll_write_mask(mask, mask);       // all rise
        while (dedicCycleCount() - start < t0 + t0h) { }
        dedic_gpio_cpu_ll_write_mask(mask, ones[bit]);  // zeros fall
        while (dedicCycleCount() - start < t0 + t1h) { }
        dedic_gpio_cpu_ll_write_mask(mask, 0);          // ones fall
    }
    // Hold the line low for the rest of the last bit period
    while (dedicCycleCount() - start < timing->bitStart[DSHOT_FRAME_BITS]) { }

    __asm__ volatile ("wsr.ps %0; rsync" :: "a"(savedPs) : "memory");
}

static void dshotDedicPostInit(void)
{
}

static bool dshotDedicEnable(void)
{
    dedicEnabled = true;
    return true;
}

static void dshotDedicDisable(void)
{
    dedicEnabled = false;
}

static bool dshotDedicIsMotorEnabled(unsigned index)
{
    return index < dedicMotorCount && dedicEnabled;
}

static bool dshotDedicDecodeTelemetry(void)
{
    return true;
}

static bool dshotDedicTelemetryWait(void)
{
    return true;
}

static void dshotDedicUpdateInit(void)
{
}

static void dshotDedicWriteInt(uint8_t index, uint16_t value)
{
    if (index >= dedicMotorCount) {
        return;
    }

    dshotDedicMotor_t *motor = &dedicMotors[index];

    // A queued DShot command replaces the throttle value
    if (dshotCommandIsProcessing()) {
        value = dshotCommandGetCurrent(index);
        if (value) {
            motor->protocolControl.requestTelemetry = true;
        }
    }

    motor->protocolControl.value = value;
    motor->packet = prepareDshotPacket(&motor->protocolControl);
}

static void dshotDedicWrite(uint8_t index, float value)
{
    dshotDedicWriteInt(index, lrintf(value));
}

static void dshotDedicUpdateComplete(void)
{
    if (!dedicEnabled) {
        return;
    }

    uint32_t ones[DSHOT_FRAME_BITS];
    for (unsigned bit = 0; bit < DSHOT_FRAME_BITS; bit++) {
        uint32_t bitMask = 0;
        for (unsigned i = 0; i < dedicMotorCount; i++) {
            bitMask |= (uint32_t)((dedicMotors[i].packet >> (DSHOT_FRAME_BITS - 1 - bit)) & 1U) << i;
        }
        ones[bit] = bitMask;
    }

    dshotDedicSendFrame(&dedicTiming, ones);
}

static void dshotDedicShutdown(void)
{
    dedicEnabled = false;
    dedic_gpio_cpu_ll_write_mask(dedicTiming.channelMask, 0);
}

static bool dshotDedicIsMotorIdle(unsigned index)
{
    // Frames are sent synchronously, so the line is always idle between calls
    UNUSED(index);
    return true;
}

static IO_t dshotDedicGetMotorIO(unsigned index)
{
    if (index >= dedicMotorCount) {
        return IO_NONE;
    }
    return dedicMotors[index].io;
}

static void dshotDedicRequestTelemetry(unsigned index)
{
    if (index < dedicMotorCount) {
        dedicMotors[index].protocolControl.requestTelemetry = true;
    }
}

static const motorVTable_t dshotDedicVTable = {
    .postInit = dshotDedicPostInit,
    .convertExternalToMotor = dshotConvertFromExternal,
    .convertMotorToExternal = dshotConvertToExternal,
    .enable = dshotDedicEnable,
    .disable = dshotDedicDisable,
    .isMotorEnabled = dshotDedicIsMotorEnabled,
    .telemetryWait = dshotDedicTelemetryWait,
    .decodeTelemetry = dshotDedicDecodeTelemetry,
    .updateInit = dshotDedicUpdateInit,
    .write = dshotDedicWrite,
    .writeInt = dshotDedicWriteInt,
    .updateComplete = dshotDedicUpdateComplete,
    .shutdown = dshotDedicShutdown,
    .isMotorIdle = dshotDedicIsMotorIdle,
    .getMotorIO = dshotDedicGetMotorIO,
    .requestTelemetry = dshotDedicRequestTelemetry,
};

static void dshotDedicSetTiming(uint32_t bitRateHz)
{
    // Each edge is computed from the frame start so that a bit period that is
    // not a whole number of cycles (e.g. 266.67 at 80MHz) does not accumulate.
    for (unsigned bit = 0; bit <= DSHOT_FRAME_BITS; bit++) {
        dedicTiming.bitStart[bit] = (uint32_t)(((uint64_t)bit * SystemCoreClock) / bitRateHz);
    }
    dedicTiming.t0h = (uint32_t)(((uint64_t)SystemCoreClock * 3) / ((uint64_t)bitRateHz * 8));   // 37.5%
    dedicTiming.t1h = (uint32_t)(((uint64_t)SystemCoreClock * 3) / ((uint64_t)bitRateHz * 4));   // 75%
}

bool dshotDedicGpioDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig)
{
    // The dedic_gpio_ll bus clock macros require this in scope
    int __DECLARE_RCC_ATOMIC_ENV __attribute__((unused));

    switch (motorConfig->motorProtocol) {
    case MOTOR_PROTOCOL_DSHOT150:
        dshotDedicSetTiming(150000);
        break;
    case MOTOR_PROTOCOL_DSHOT600:
        dshotDedicSetTiming(600000);
        break;
    case MOTOR_PROTOCOL_DSHOT300:
    default:
        dshotDedicSetTiming(300000);
        break;
    }

    dedic_gpio_ll_enable_bus_clock(true);
    dedic_gpio_ll_reset_register();

    dedicMotorCount = 0;
    dedicEnabled = false;

    for (int i = 0; i < DSHOT_DEDIC_MAX_MOTORS && i < MAX_SUPPORTED_MOTORS; i++) {
        const ioTag_t tag = motorConfig->ioTags[i];
        if (!tag) {
            break;
        }

        IO_t io = IOGetByTag(tag);
        if (!io) {
            break;
        }

        dedicMotors[i].io = io;
        dedicMotors[i].packet = 0;
        dedicMotors[i].protocolControl.value = 0;
        dedicMotors[i].protocolControl.requestTelemetry = false;

        dedicMotorCount++;
    }

    if (dedicMotorCount == 0) {
        return false;
    }

    // Drive every channel low before any pin is connected to it: DShot idles low
    dedicTiming.channelMask = (1U << dedicMotorCount) - 1;
    dedic_gpio_cpu_ll_write_mask(dedicTiming.channelMask, 0);

    for (unsigned i = 0; i < dedicMotorCount; i++) {
        IO_t io = dedicMotors[i].io;
        const uint32_t pin = IO_Pin(io);

        IOInit(io, OWNER_MOTOR, RESOURCE_INDEX(i));
        IOConfigGPIO(io, IOCFG_OUT_PP);

        // Route core-0 dedicated GPIO channel i to the pin via the GPIO matrix
        esp_rom_gpio_pad_select_gpio(pin);
        gpio_ll_output_enable(&GPIO, pin);
        esp_rom_gpio_connect_out_signal(pin, PRO_ALONEGPIO_OUT0_IDX + i, false, false);
    }

    device->vTable = &dshotDedicVTable;
    device->count = dedicMotorCount;
    device->initialized = true;

    dshotMotorCount = dedicMotorCount;

    return true;
}

#endif // USE_DSHOT && USE_DSHOT_DEDIC_GPIO
