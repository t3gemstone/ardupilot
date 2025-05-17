#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_LSM6DSO : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation);

    void start() override;
    bool update() override;
    float get_temperature();

private:
    AP_InertialSensor_LSM6DSO(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::Device> dev,
                             enum Rotation rotation);

    bool _init();
    bool _hardware_init();
    void _poll_data();
    void _set_gyro_scale();
    void _set_accel_scale();

    // Register operations
    void _write_register(uint8_t reg, uint8_t val);
    uint8_t _read_register(uint8_t reg);
    void _read_registers(uint8_t first_reg, uint8_t *recv, uint8_t len);

    // Configuration
    void _configure_accel();
    void _configure_gyro();
    void _configure_int();
    void _soft_reset();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    enum Rotation _rotation;
    float _accel_scale;
    float _gyro_scale;
    uint16_t _accel_odr;
    uint16_t _gyro_odr;
};
