/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef AP_Compass_MPU9150_Slave_H
#define AP_Compass_MPU9150_Slave_H

#include "Compass.h"
#include "../AP_InertialSensor/AP_InertialSensor_MPU9150.h"

class AP_Compass_MPU9150_Slave : public Compass
{
public:
    AP_Compass_MPU9150_Slave(AP_InertialSensor_MPU9150 &ins) : 
        Compass(),
        _ins(ins)
    {
        product_id = AP_COMPASS_TYPE_MPU9150_SLAVE;
        _num_instances = 0;
    }
    bool        init(void);
    bool        read(void);
    void        accumulate(void);

    // return the number of compass instances
    uint8_t get_count(void) const { return _num_instances; }

    // return the primary compass
    uint8_t get_primary(void) const { return 0; }

private:
    int16_t setup_compass(void);
    int16_t get_compass_reg(int16_t *data);

    // Ref-to-pointer of MPU9150 inertial sensor
    AP_InertialSensor_MPU9150   &_ins;

    bool            _initialized;

    AP_HAL::Semaphore* _i2c_sem;

    uint8_t         _compass_addr;
    uint16_t        _compass_sample_rate;
    int16_t         _mag_sens_adj[3];

    uint8_t _num_instances;
    Vector3f _sum[COMPASS_MAX_INSTANCES];
    uint32_t _count[COMPASS_MAX_INSTANCES];
    uint64_t _last_timestamp[COMPASS_MAX_INSTANCES];
};

#endif // AP_Compass_MPU9150_Slave_H

