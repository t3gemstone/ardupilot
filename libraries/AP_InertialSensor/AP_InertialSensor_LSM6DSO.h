#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

#define FIFO_BATCH_SIZE 32
#define FIFO_SAMPLE_BYTES 7

class AP_InertialSensor_LSM6DSO : public AP_InertialSensor_Backend
{
public:
    virtual ~AP_InertialSensor_LSM6DSO() { }
    void start() override;
    bool update() override;

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation);

private:
    AP_InertialSensor_LSM6DSO(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev,
                              enum Rotation rotation);


    bool _init_sensor();
    bool _hardware_init();

    bool _block_read(uint8_t reg, uint8_t *buf, uint32_t len);
    uint8_t _register_read(uint8_t reg);
    void _register_write(uint8_t reg, uint8_t val);

    void _soft_reset();
    void _gyro_init();
    void _accel_init();
    void _fifo_init();

    void _set_gyro_scale();
    void _set_accel_scale();

    void _read_gyro_data(uint8_t* gyro_data);
    void _read_accel_data(uint8_t* accel_data);
    void _poll_data();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    enum Rotation _rotation;
    float _accel_scale;
    float _gyro_scale;

    uint8_t fifo_buf[FIFO_BATCH_SIZE * FIFO_SAMPLE_BYTES];
};
