#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_InertialSensor_LSM6DSO.h"

extern const AP_HAL::HAL& hal;

#define LSM6DSO_WHOAMI 0x6C

#define LSM6DSO_REG_WHO_AM_I   0x0F
#define LSM6DSO_REG_CTRL1_XL   0x10
#define LSM6DSO_REG_CTRL2_G    0x11
#define LSM6DSO_REG_CTRL3_C    0x12
#define LSM6DSO_REG_STATUS_REG 0x1E

// Accelerometer output registers
#define LSM6DSO_REG_OUTX_L_A   0x28
#define LSM6DSO_REG_OUTX_H_A   0x29
#define LSM6DSO_REG_OUTY_L_A   0x2A
#define LSM6DSO_REG_OUTY_H_A   0x2B
#define LSM6DSO_REG_OUTZ_L_A   0x2C
#define LSM6DSO_REG_OUTZ_H_A   0x2D

// Temperature output registers
#define LSM6DSO_REG_OUT_TEMP_L 0x20
#define LSM6DSO_REG_OUT_TEMP_H 0x21

// Gyroscope output registers
#define LSM6DSO_REG_OUTX_L_G   0x22
#define LSM6DSO_REG_OUTX_H_G   0x23
#define LSM6DSO_REG_OUTY_L_G   0x24
#define LSM6DSO_REG_OUTY_H_G   0x25
#define LSM6DSO_REG_OUTZ_L_G   0x26
#define LSM6DSO_REG_OUTZ_H_G   0x27

// CTRL3_C configuration
#define LSM6DSO_REG_CTRL3_C_BDU      (1 << 6)
#define LSM6DSO_REG_CTRL3_C_IF_INC   (1 << 2)

// Accelerometer configuration
#define LSM6DSO_ACCEL_FS_8G  0x03
#define LSM6DSO_ACCEL_ODR_833HZ 0x07

// Gyroscope configuration
#define LSM6DSO_GYRO_FS_2000DPS 0x03
#define LSM6DSO_GYRO_ODR_833HZ 0x07

AP_InertialSensor_LSM6DSO::AP_InertialSensor_LSM6DSO(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                   enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _rotation(rotation)
{
}

AP_InertialSensor_Backend *AP_InertialSensor_LSM6DSO::probe(
    AP_InertialSensor &imu,
    AP_HAL::OwnPtr<AP_HAL::Device> dev,
    enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    auto sensor = new AP_InertialSensor_LSM6DSO(imu, std::move(dev), rotation);
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_InertialSensor_LSM6DSO::_init()
{
    if (!_dev->get_semaphore()->take(10)) {
        return false;
    }

    // Check WHOAMI
    uint8_t whoami = _read_register(LSM6DSO_REG_WHO_AM_I);
    if (whoami != LSM6DSO_WHOAMI) {
        AP_HAL::panic("LSM6DSO: Bad WHOAMI 0x%02x", whoami);
    }

    // Initialize sensor
    if (!_hardware_init()) {
        _dev->get_semaphore()->give();
        return false;
    }

    _dev->get_semaphore()->give();
    return true;
}

bool AP_InertialSensor_LSM6DSO::_hardware_init()
{
    _soft_reset();
    hal.scheduler->delay(100);

    // Configure accelerometer
    _configure_accel();

    // Configure gyroscope
    _configure_gyro();

    // Configure interrupts and BDU
    _configure_int();

    // Add delay after configuration
    hal.scheduler->delay(50);

    _set_accel_scale();
    _set_gyro_scale();

    return true;
}

void AP_InertialSensor_LSM6DSO::_soft_reset()
{
    _write_register(LSM6DSO_REG_CTRL3_C, 0x01);
    hal.scheduler->delay(100);
}

// Update _configure_accel() to track ODR
void AP_InertialSensor_LSM6DSO::_configure_accel()
{
    // 833Hz ODR, 8G range
    _write_register(LSM6DSO_REG_CTRL1_XL,
                   (LSM6DSO_ACCEL_ODR_833HZ << 4) |
                   (LSM6DSO_ACCEL_FS_8G << 2));
    _accel_odr = 833; // Add this line to track ODR
}

void AP_InertialSensor_LSM6DSO::_configure_gyro()
{
    // 833Hz ODR, 2000DPS range
    _write_register(LSM6DSO_REG_CTRL2_G,
                   (LSM6DSO_GYRO_ODR_833HZ << 4) |
                   (LSM6DSO_GYRO_FS_2000DPS << 2));
    _gyro_odr = 833; // Add this line to track ODR
}

void AP_InertialSensor_LSM6DSO::_configure_int()
{
    // Enable block data update and auto-increment
    _write_register(LSM6DSO_REG_CTRL3_C,
                   LSM6DSO_REG_CTRL3_C_BDU |
                   LSM6DSO_REG_CTRL3_C_IF_INC);
}

void AP_InertialSensor_LSM6DSO::_set_accel_scale()
{
    // Correct scale for 8G range (32768 counts)
    _accel_scale = (8.0f * GRAVITY_MSS) / 32768.0f;
}

void AP_InertialSensor_LSM6DSO::_set_gyro_scale()
{
    // Correct scale for 2000DPS range (32768 counts)
    // Convert DPS to rad/s: 2000 * (pi/180) = 2000 * 0.01745329251f
    _gyro_scale = (2000.0f * 0.01745329251f) / 32768.0f;
}

float AP_InertialSensor_LSM6DSO::get_temperature()
{
    if (!_dev->get_semaphore()->take(10)) {
        return 0.0f;
    }

    uint8_t temp_data[2];
    _read_registers(LSM6DSO_REG_OUT_TEMP_L, temp_data, 2);
    _dev->get_semaphore()->give();

    int16_t temp_raw = (int16_t)((temp_data[1] << 8) | temp_data[0]);
    return (temp_raw / 256.0f) + 25.0f; // Conversion per datasheet
}

void AP_InertialSensor_LSM6DSO::start()
{
    // Register with actual ODR (833Hz)
    if (!_imu.register_accel(accel_instance, _accel_odr, _dev->get_bus_id_devtype(DEVTYPE_INS_LSM6DSO)) ||
        !_imu.register_gyro(gyro_instance, _gyro_odr, _dev->get_bus_id_devtype(DEVTYPE_INS_LSM6DSO))) {
        return;
    }

    // Adjust polling interval to 1200µs (833Hz)
    _dev->register_periodic_callback(1200,
        FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM6DSO::_poll_data, void));
}

bool AP_InertialSensor_LSM6DSO::update()
{
    // Push to sample queue
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

void AP_InertialSensor_LSM6DSO::_poll_data()
{
    uint8_t status = _read_register(LSM6DSO_REG_STATUS_REG);

    // Check both accel and gyro data ready bits (bits 0 and 1)
    if (!(status & 0x03)) {
        return;
    }

    // Read accelerometer data
    uint8_t accel_data[6];
    _read_registers(LSM6DSO_REG_OUTX_L_A, accel_data, sizeof(accel_data));

    Vector3f accel(
        (int16_t)((accel_data[1] << 8) | accel_data[0]),  // X (little-endian)
        (int16_t)((accel_data[3] << 8) | accel_data[2]),  // Y
        (int16_t)((accel_data[5] << 8) | accel_data[4])); // Z
    accel *= _accel_scale;

    // Read gyro data
    uint8_t gyro_data[6];
    _read_registers(LSM6DSO_REG_OUTX_L_G, gyro_data, sizeof(gyro_data));

    Vector3f gyro(
        (int16_t)((gyro_data[1] << 8) | gyro_data[0]),  // X (little-endian)
        (int16_t)((gyro_data[3] << 8) | gyro_data[2]),  // Y
        (int16_t)((gyro_data[5] << 8) | gyro_data[4])); // Z
    gyro *= _gyro_scale;

    // Apply rotation
    accel.rotate(_rotation);
    gyro.rotate(_rotation);

    _rotate_and_correct_accel(accel_instance, accel);
    _rotate_and_correct_gyro(gyro_instance, gyro);

    _notify_new_accel_raw_sample(accel_instance, accel);
    _notify_new_gyro_raw_sample(gyro_instance, gyro);
}

void AP_InertialSensor_LSM6DSO::_write_register(uint8_t reg, uint8_t val)
{
    _dev->write_register(reg, val);
}

uint8_t AP_InertialSensor_LSM6DSO::_read_register(uint8_t reg)
{
    uint8_t val = 0;
    _dev->read_registers(reg, &val, 1);
    return val;
}

void AP_InertialSensor_LSM6DSO::_read_registers(uint8_t first_reg, uint8_t *recv, uint8_t len)
{
    _dev->read_registers(first_reg, recv, len);
}
