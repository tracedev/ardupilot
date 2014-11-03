/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_Compass_AK8975.cpp - AK8975 Magnetometer driver
 *       Tested with MPU9150; requires I2C slave bus of MPU9150 be put in bypass,
 *       allowing the CPU I2C bus to access the AK8975
 *
 */


#include <AP_HAL.h>

// Define if using an AK8975 on the slave I2C bus of the MPU9150
// Requires that the MPU9150 InertialSensor driver is being used
#define MPU9150_BYPASS

#include "AP_Compass_AK8975.h"
#ifdef MPU9150_BYPASS
#include "AP_InertialSensor_MPU9150.h"
#endif
#include <stdio.h>

#define AK89xx_FSR (9830)

extern const AP_HAL::HAL& hal;

#ifdef MPU9150_BYPASS
extern AP_InertialSensor_MPU9150 ins;
#define AK8975_ACCESSIBLE() ins.mpu_get_bypass()
#else
#define AK8975_ACCESSIBLE() true
#endif

// AK8975_SECONDARY
#define SUPPORTS_AK89xx_HIGH_SENS   (0x00)
#define AK89xx_FSR                  (9830)

// AK89xx_SECONDARY
#define AKM_REG_WHOAMI      (0x00)

#define AKM_REG_ST1         (0x02)
#define AKM_REG_HXL         (0x03)
#define AKM_REG_HXH         (0x04)
#define AKM_REG_HYL         (0x05)
#define AKM_REG_HYH         (0x06)
#define AKM_REG_HZL         (0x07)
#define AKM_REG_HZH         (0x08)
#define AKM_REG_ST2         (0x09)

#define AKM_REG_CNTL        (0x0A)
#define AKM_REG_ASTC        (0x0C)
#define AKM_REG_ASAX        (0x10)
#define AKM_REG_ASAY        (0x11)
#define AKM_REG_ASAZ        (0x12)

#define AKM_DATA_READY      (0x01)
#define AKM_DATA_OVERRUN    (0x02)
#define AKM_OVERFLOW        (0x80)
#define AKM_DATA_ERROR      (0x40)

#define AKM_BIT_SELF_TEST   (0x40)

#define AKM_POWER_DOWN          (0x00 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_SINGLE_MEASUREMENT  (0x01 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_FUSE_ROM_ACCESS     (0x0F | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_MODE_SELF_TEST      (0x08 | SUPPORTS_AK89xx_HIGH_SENS)

#define AKM_WHOAMI      (0x48)

#define MAX_COMPASS_SAMPLE_RATE (100)

// Public Methods //////////////////////////////////////////////////////////////

/**
 *  @brief      Initialize the compass
 *  @return     true regardless of device setup state
 */
bool AP_Compass_AK8975::init(void)
{
    _num_instances = 1;

    _external[0] = 0;
    _count[0] = 0;
    _sum[0].zero();
    _healthy[0] = false;

    // get pointer to i2c bus semaphore
    _i2c_sem = hal.i2c->get_semaphore();

    if (!_i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.scheduler->panic(PSTR("Failed to get Compass_AK8975 semaphore"));
    }

    // We may not be able to initialize here, depending on if the INS has initialized
    // the AP_InertialSensor_MPU9150 already to set the I2C bus in bypass mode
    // Therefore, we need to retry later if we don't initialize
    if (AK8975_ACCESSIBLE() && 0 == setup_compass()) {
        _initialized = true;
        _healthy[0] = true;
        // give the driver a chance to run, and gather one sample
        hal.scheduler->delay(40);
        accumulate();
        if (_count[0] == 0) {
            hal.console->printf("Failed initial compass accumulate\n");        
        }
    } else {
#ifndef MPU9150_BYPASS
        hal.console->printf("Deferring compass init, device not ready\n");
#endif
    }

    _i2c_sem->give();

    return true;
}

/**
 *  @brief      Read accumulated data
 *  @return     true if device is healthy
 */
bool AP_Compass_AK8975::read(void)
{
    // try to accumulate one more sample, so we have the latest data
    accumulate();

    // consider the compass healthy if we got a reading in the last 0.2s
    _healthy[0] = (hal.scheduler->micros64() - _last_timestamp[0] < 200000);

    // avoid division by zero if we haven't received any mag reports
    if (_count[0]) {

        _sum[0] /= _count[0];
        _sum[0] *= 1000;

        // apply default board orientation for this compass type. This is
        // a noop on most boards
        _sum[0].rotate(MAG_BOARD_ORIENTATION);

        if (_external[0]) {
            // add user selectable orientation
            _sum[0].rotate((enum Rotation)_orientation[0].get());
        } else {
            // add in board orientation from AHRS
            _sum[0].rotate(_board_orientation);
        }

        _sum[0] += _offset[0].get();

        // apply motor compensation
        if (_motor_comp_type != AP_COMPASS_MOT_COMP_DISABLED && _thr_or_curr != 0.0f) {
            _motor_offset[0] = _motor_compensation[0].get() * _thr_or_curr;
            _sum[0] += _motor_offset[0];
        } else {
            _motor_offset[0].zero();
        }

        _field[0] = _sum[0];

        _sum[0].zero();
        _count[0] = 0;

        last_update = _last_timestamp[0];
    }
    
    return _healthy[0];
}

/**
 *  @brief      Obtain a reading from the AK8975
 *              Will attempt initialization if not already
 *              complete
 */
void AP_Compass_AK8975::accumulate(void)
{
    Vector3f mag_data;
    int16_t data[3];
    int16_t ret;

    if (!_initialized) {
        if (AK8975_ACCESSIBLE()) {
            // We may not have been able to initialize yet, try here
            if (!_i2c_sem->take_nonblocking()) {
                return;
            }
            if (0 == setup_compass()) {
                 _i2c_sem->give();
                 _healthy[0] = true;
                _initialized = true;
            } else {
                _i2c_sem->give();
                hal.console->printf("Failed compass setup in accumulate\n");
                return;
            }
        } else {
            return;
        }
    }

    // take i2c bus sempahore
    if (!_i2c_sem->take(1)) {
        return;
    }

    ret = get_compass_reg(data);
    _i2c_sem->give();

    if (0 == ret) {
        mag_data = Vector3f(data[0], data[1], data[2]);
        _healthy[0] = true;
        _sum[0] += mag_data;
        _count[0]++;
        _last_timestamp[0] = hal.scheduler->micros64();
    }

}

/**
 *  @brief      Configure the AK8975
 *              The AK8975 must already be in bypass mode for
 *              setup to succeed
 *  @return     0 if successful.
 */
int16_t AP_Compass_AK8975::setup_compass(void)
{
    uint8_t data[4], akm_addr;

    /* Find compass. Possible addresses range from 0x0C to 0x0F. */
    for (akm_addr = 0x0C; akm_addr <= 0x0F; akm_addr++) {
        uint8_t result;
        result = hal.i2c->readRegister(akm_addr, AKM_REG_WHOAMI, data);
        if (!result && (data[0] == AKM_WHOAMI))
            break;
    }

    if (akm_addr > 0x0F) {
        // We may not be able to find the compass if the AK8975 hasn't been
        // configured in bypass mode yet
        hal.console->printf("AP_Compass_AK8975: Compass not found.\n");
        return -1;
    } else {
        hal.console->printf("Found compass at 0x%02x\n", akm_addr);
    }

    _compass_addr = akm_addr;

    data[0] = AKM_POWER_DOWN;
    if (hal.i2c->writeRegister(_compass_addr, AKM_REG_CNTL, data[0]))
        return -1;
    hal.scheduler->delay(1);

    data[0] = AKM_FUSE_ROM_ACCESS;
    if (hal.i2c->writeRegister(_compass_addr, AKM_REG_CNTL, data[0]))
        return -1;
    hal.scheduler->delay(1);

    /* Get sensitivity adjustment data from fuse ROM. */
    if (hal.i2c->readRegisters(_compass_addr, AKM_REG_ASAX, 3, data))
        return -1;
    _mag_sens_adj[0] = (long)data[0] + 128;
    _mag_sens_adj[1] = (long)data[1] + 128;
    _mag_sens_adj[2] = (long)data[2] + 128;

    data[0] = AKM_POWER_DOWN;
    if (hal.i2c->writeRegister(_compass_addr, AKM_REG_CNTL, data[0]))
        return -1;
    hal.scheduler->delay(1);

    // Prime the pump for a first measurement
    data[0] = AKM_SINGLE_MEASUREMENT;
    if (hal.i2c->writeRegister(_compass_addr, AKM_REG_CNTL, data[0]))
        return -1;
    hal.scheduler->delay(1);

    return 0;
}

/**
 *  @brief      Read raw compass data from device
 *              This reads the most recent stored measurement
 *              and initiates another single measurement
 *  @param[out] data        Raw data in hardware units.
 *  @return     0 if successful.
 */
int16_t AP_Compass_AK8975::get_compass_reg(int16_t *data)
{
    uint8_t tmp[9];

    if (hal.i2c->readRegisters(_compass_addr, AKM_REG_ST1, 8, tmp))
        return -1;
    tmp[8] = AKM_SINGLE_MEASUREMENT;
    if (hal.i2c->writeRegister(_compass_addr, AKM_REG_CNTL, tmp[8]))
        return -1;

    /* AK8975 doesn't have the overrun error bit. */
    if (!(tmp[0] & AKM_DATA_READY))
        return -2;
    if ((tmp[7] & AKM_OVERFLOW) || (tmp[7] & AKM_DATA_ERROR))
        return -3;

    data[0] = (tmp[2] << 8) | tmp[1];
    data[1] = (tmp[4] << 8) | tmp[3];
    data[2] = (tmp[6] << 8) | tmp[5];

    data[0] = ((long)data[0] * _mag_sens_adj[0]) >> 8;
    data[1] = ((long)data[1] * _mag_sens_adj[1]) >> 8;
    data[2] = ((long)data[2] * _mag_sens_adj[2]) >> 8;

    return 0;
}

