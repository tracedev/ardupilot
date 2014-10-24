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
 *       AP_Compass_MPU9150_Slave.cpp - MPU9150 Magnetometer to pair with AP_InertialSensor_MPU9150
 *       AP_InertialSensor_MPU9150 configures the MPU9150 as an I2C master on the auxiliary I2C bus
 *       It also sets the MPU9150 to poll the built-in compass (AK8975) on the auxiliary bus and
 *       place it in the Slave 0 registers
 *       This compass driver obtains the latest data from these Slave 0 registers by calling
 *       the AP_InertialSensor_MPU9150 driver
 *
 */


#include <AP_HAL.h>

#include "AP_Compass_MPU9150_Slave.h"
#include "../AP_InertialSensor/AP_InertialSensor_MPU9150.h"

#define AK89xx_FSR (9830)

extern const AP_HAL::HAL& hal;


// Public Methods //////////////////////////////////////////////////////////////

bool AP_Compass_MPU9150_Slave::init(void)
{
    _num_instances = 1;

    _external[0] = 0;
    _count[0] = 0;
    _sum[0].zero();
    _healthy[0] = false;

    // give the driver a chance to run, and gather one sample
    hal.scheduler->delay(40);
    accumulate();
    if (_count[0] == 0) {
        hal.console->printf("Failed initial compass accumulate\n");        
    }
    return true;
}

bool AP_Compass_MPU9150_Slave::read(void)
{
    // try to accumulate one more sample, so we have the latest data
    accumulate();

    // consider the compass healthy if we got a reading in the last 0.2s
    //_healthy[0] = (hal.scheduler->micros64() - _last_timestamp[0] < 200000);
    _healthy[0] = true;

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

void AP_Compass_MPU9150_Slave::accumulate(void)
{
    // Get the sample from the AP_InertialSensor_MPU9150 driver
    Vector3f mag_data;
    if (0 == _ins.read_compass(mag_data)) {
        _sum[0] += mag_data;
        _count[0]++;
        _last_timestamp[0] = hal.scheduler->micros64();
    }

}

