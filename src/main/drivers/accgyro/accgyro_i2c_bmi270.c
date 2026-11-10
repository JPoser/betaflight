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
 * BMI270 on I2C.
 *
 * The SPI driver (accgyro_spi_bmi270.c) samples at 3.2kHz from a data ready
 * interrupt. I2C cannot keep up with that: a six byte register read is about
 * 85 bit times on the wire, i.e. 0.2ms at 400kHz and 0.65ms at 133kHz, and
 * the read blocks the gyro task. This driver therefore runs the sensor at a
 * much lower output data rate (BMI270_I2C_GYRO_RATE_HZ) and polls it, with no
 * interrupt pin.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_ACCGYRO_BMI270) && defined(USE_I2C_GYRO)

#include "common/axis.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_i2c_bmi270.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/time.h"

#define BMI270_CONFIG_SIZE 328

// Device config (microcode) that must be uploaded to the sensor
extern const uint8_t bmi270_maximum_fifo_config_file[BMI270_CONFIG_SIZE];

#define BMI270_CHIP_ID 0x24

// The config is uploaded in bursts. A burst must be an even number of bytes and,
// with the address and register bytes, fit the smallest I2C transmit FIFO among
// the supported MCUs (32 bytes on ESP32).
#define BMI270_I2C_UPLOAD_CHUNK 16U

typedef enum {
    BMI270_REG_CHIP_ID = 0x00,
    BMI270_REG_ACC_DATA_X_LSB = 0x0C,
    BMI270_REG_GYR_DATA_X_LSB = 0x12,
    BMI270_REG_INTERNAL_STATUS = 0x21,
    BMI270_REG_ACC_CONF = 0x40,
    BMI270_REG_ACC_RANGE = 0x41,
    BMI270_REG_GYRO_CONF = 0x42,
    BMI270_REG_GYRO_RANGE = 0x43,
    BMI270_REG_INIT_CTRL = 0x59,
    BMI270_REG_INIT_ADDR_0 = 0x5B,
    BMI270_REG_INIT_ADDR_1 = 0x5C,
    BMI270_REG_INIT_DATA = 0x5E,
    BMI270_REG_PWR_CONF = 0x7C,
    BMI270_REG_PWR_CTRL = 0x7D,
    BMI270_REG_CMD = 0x7E,
} bmi270I2cRegister_e;

typedef enum {
    BMI270_VAL_CMD_SOFTRESET = 0xB6,
    BMI270_VAL_PWR_CTRL = 0x0E,                 // enable gyro, acc and temp sensors
    BMI270_VAL_PWR_CONF = 0x02,                 // disable advanced power save, enable FIFO self-wake
    BMI270_VAL_ACC_CONF_ODR200 = 0x09,          // set acc sample rate to 200hz
    BMI270_VAL_ACC_CONF_BWP = 0x02,             // set acc filter in normal mode
    BMI270_VAL_ACC_CONF_HP = 0x01,              // set acc in high performance mode
    BMI270_VAL_ACC_RANGE_16G = 0x03,            // set acc to 16G full scale
    BMI270_VAL_GYRO_CONF_ODR400 = 0x0A,         // set gyro sample rate to 400hz
    BMI270_VAL_GYRO_CONF_ODR800 = 0x0B,         // set gyro sample rate to 800hz
    BMI270_VAL_GYRO_CONF_ODR1600 = 0x0C,        // set gyro sample rate to 1600hz
    BMI270_VAL_GYRO_CONF_BWP_NORM = 0x02,       // set gyro filter in normal mode
    BMI270_VAL_GYRO_CONF_NOISE_PERF = 0x01,     // set gyro in high performance noise mode
    BMI270_VAL_GYRO_CONF_FILTER_PERF = 0x01,    // set gyro in high performance filter mode
    BMI270_VAL_GYRO_RANGE_2000DPS = 0x08,       // 2000dps full scale, as for the SPI driver
    BMI270_VAL_INTERNAL_STATUS_INIT_OK = 0x01,  // low nibble of INTERNAL_STATUS after a good upload
} bmi270I2cConfigValues_e;

#if BMI270_I2C_GYRO_RATE_HZ == 400
#define BMI270_VAL_GYRO_CONF_ODR BMI270_VAL_GYRO_CONF_ODR400
#elif BMI270_I2C_GYRO_RATE_HZ == 800
#define BMI270_VAL_GYRO_CONF_ODR BMI270_VAL_GYRO_CONF_ODR800
#elif BMI270_I2C_GYRO_RATE_HZ == 1600
#define BMI270_VAL_GYRO_CONF_ODR BMI270_VAL_GYRO_CONF_ODR1600
#else
#error "BMI270_I2C_GYRO_RATE_HZ must be 400, 800 or 1600"
#endif

static void bmi270I2cRegisterWrite(const extDevice_t *dev, bmi270I2cRegister_e registerId, uint8_t value, unsigned delayMs)
{
    busWriteRegister(dev, registerId, value);
    if (delayMs) {
        delay(delayMs);
    }
}

bool bmi270I2cDetect(const extDevice_t *dev)
{
    uint8_t chipId = 0;

    return busReadRegisterBuffer(dev, BMI270_REG_CHIP_ID, &chipId, 1) && chipId == BMI270_CHIP_ID;
}

static bool bmi270I2cUploadConfig(const extDevice_t *dev)
{
    bmi270I2cRegisterWrite(dev, BMI270_REG_PWR_CONF, 0, 1);
    bmi270I2cRegisterWrite(dev, BMI270_REG_INIT_CTRL, 0, 1);

    // Each burst is preceded by its position in the config file, counted in 16 bit words
    for (unsigned offset = 0; offset < BMI270_CONFIG_SIZE; offset += BMI270_I2C_UPLOAD_CHUNK) {
        const unsigned wordIndex = offset / 2;
        const unsigned length = MIN(BMI270_I2C_UPLOAD_CHUNK, BMI270_CONFIG_SIZE - offset);

        bmi270I2cRegisterWrite(dev, BMI270_REG_INIT_ADDR_0, wordIndex & 0x0F, 0);
        bmi270I2cRegisterWrite(dev, BMI270_REG_INIT_ADDR_1, (wordIndex >> 4) & 0xFF, 0);

        if (!i2cWriteBuffer(dev->bus->busType_u.i2c.device, dev->busType_u.i2c.address, BMI270_REG_INIT_DATA, length, (uint8_t *)&bmi270_maximum_fifo_config_file[offset])) {
            return false;
        }
    }

    delay(10);
    bmi270I2cRegisterWrite(dev, BMI270_REG_INIT_CTRL, 1, 20);

    uint8_t status = 0;
    return busReadRegisterBuffer(dev, BMI270_REG_INTERNAL_STATUS, &status, 1)
        && (status & 0x0F) == BMI270_VAL_INTERNAL_STATUS_INIT_OK;
}

static void bmi270I2cConfig(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;

    // Perform a soft reset to set all configuration to default
    // Delay 100ms before continuing configuration
    bmi270I2cRegisterWrite(dev, BMI270_REG_CMD, BMI270_VAL_CMD_SOFTRESET, 100);

    // A failed upload leaves the sensor returning zeros; one retry covers a glitched burst
    if (!bmi270I2cUploadConfig(dev)) {
        bmi270I2cRegisterWrite(dev, BMI270_REG_CMD, BMI270_VAL_CMD_SOFTRESET, 100);
        bmi270I2cUploadConfig(dev);
    }

    // Configure the accelerometer
    bmi270I2cRegisterWrite(dev, BMI270_REG_ACC_CONF, (BMI270_VAL_ACC_CONF_HP << 7) | (BMI270_VAL_ACC_CONF_BWP << 4) | BMI270_VAL_ACC_CONF_ODR200, 1);

    // Configure the accelerometer full-scale range
    bmi270I2cRegisterWrite(dev, BMI270_REG_ACC_RANGE, BMI270_VAL_ACC_RANGE_16G, 1);

    // Configure the gyro. At these low data rates the sensor's own filter is left
    // in normal mode: the oversampling modes would add delay.
    bmi270I2cRegisterWrite(dev, BMI270_REG_GYRO_CONF, (BMI270_VAL_GYRO_CONF_FILTER_PERF << 7) | (BMI270_VAL_GYRO_CONF_NOISE_PERF << 6) | (BMI270_VAL_GYRO_CONF_BWP_NORM << 4) | BMI270_VAL_GYRO_CONF_ODR, 1);

    // Configure the gyro full-range scale
    bmi270I2cRegisterWrite(dev, BMI270_REG_GYRO_RANGE, BMI270_VAL_GYRO_RANGE_2000DPS, 1);

    // Configure the device for performance mode
    bmi270I2cRegisterWrite(dev, BMI270_REG_PWR_CONF, BMI270_VAL_PWR_CONF, 1);

    // Enable the gyro, accelerometer and temperature sensor - disable aux interface
    bmi270I2cRegisterWrite(dev, BMI270_REG_PWR_CTRL, BMI270_VAL_PWR_CTRL, 1);
}

// Sensor data is little endian, X then Y then Z
static bool bmi270I2cReadAxes(const extDevice_t *dev, bmi270I2cRegister_e registerId, int16_t *axes)
{
    uint8_t data[6];

    if (!busReadRegisterBuffer(dev, registerId, data, sizeof(data))) {
        return false;
    }

    axes[X] = (int16_t)((data[1] << 8) | data[0]);
    axes[Y] = (int16_t)((data[3] << 8) | data[2]);
    axes[Z] = (int16_t)((data[5] << 8) | data[4]);

    return true;
}

static bool bmi270I2cAccRead(accDev_t *acc)
{
    return bmi270I2cReadAxes(&acc->gyro->dev, BMI270_REG_ACC_DATA_X_LSB, acc->ADCRaw);
}

static bool bmi270I2cGyroRead(gyroDev_t *gyro)
{
    return bmi270I2cReadAxes(&gyro->dev, BMI270_REG_GYR_DATA_X_LSB, gyro->gyroADCRaw);
}

static void bmi270I2cGyroInit(gyroDev_t *gyro)
{
    bmi270I2cConfig(gyro);
}

static void bmi270I2cAccInit(accDev_t *acc)
{
    // sensor is configured during gyro init
    acc->acc_1G = 512 * 4;   // 16G sensor scale
}

bool bmi270I2cAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != BMI_270_I2C) {
        return false;
    }

    acc->initFn = bmi270I2cAccInit;
    acc->readFn = bmi270I2cAccRead;

    return true;
}

bool bmi270I2cGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != BMI_270_I2C) {
        return false;
    }

    gyro->initFn = bmi270I2cGyroInit;
    gyro->readFn = bmi270I2cGyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}

#endif // USE_ACCGYRO_BMI270 && USE_I2C_GYRO
