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
#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_LSM6DSO.h"

#include <AP_Math/AP_Math.h>

#include <stdio.h>

#define WHO_AM_I 0x6C

extern const AP_HAL::HAL& hal;

/*
 * Accelerometer and Gyroscope registers
 */
#define LSM6DSO_REG_WHO_AM_I   0x0F
#define LSM6DSO_REG_FIFO_CTRL1   0x07
#   define LSM6DSO_REG_FIFO_CTRL1_WTM_1   (1 << 0)
#define LSM6DSO_REG_FIFO_CTRL2   0x08
#define LSM6DSO_REG_FIFO_CTRL3   0x09
#   define LSM6DSO_REG_FIFO_CTRL3_BDR_XL_833HZ (0x07)
#   define LSM6DSO_REG_FIFO_CTRL3_BDR_XL_1667HZ (0x08)
#   define LSM6DSO_REG_FIFO_CTRL3_BDR_XL_3333HZ (0x09)
#   define LSM6DSO_REG_FIFO_CTRL3_BDR_XL_6667HZ (0x0A)
#   define LSM6DSO_REG_FIFO_CTRL3_BDR_GY_833HZ (0x07 << 4)
#   define LSM6DSO_REG_FIFO_CTRL3_BDR_GY_1667HZ (0x08 << 4)
#   define LSM6DSO_REG_FIFO_CTRL3_BDR_GY_3333HZ (0x09 << 4)
#   define LSM6DSO_REG_FIFO_CTRL3_BDR_GY_6667HZ (0x0A << 4)
#define LSM6DSO_REG_FIFO_CTRL4   0x0A
#   define LSM6DSO_REG_FIFO_CTRL4_FMODE_BYPASS (0x00)
#   define LSM6DSO_REG_FIFO_CTRL4_FMODE_CON (0x06)
#define LSM6DSO_REG_CTRL1_XL   0x10
#   define LSM6DSO_REG_CTRL1_XL_FS_8G     (0x03 << 2)
#   define LSM6DSO_REG_CTRL1_XL_ODR 1667
#   define LSM6DSO_REG_CTRL1_XL_ODR_833HZ (0x07 << 4)
#   define LSM6DSO_REG_CTRL1_XL_ODR_1667HZ (0x08 << 4)
#   define LSM6DSO_REG_CTRL1_XL_ODR_3333HZ (0x09 << 4)
#   define LSM6DSO_REG_CTRL1_XL_ODR_6667HZ (0x0A << 4)
#define LSM6DSO_REG_CTRL2_G    0x11
#   define LSM6DSO_REG_CTRL2_G_FS_2000DPS (0x03 << 2)
#   define LSM6DSO_REG_CTRL2_G_ODR 1667
#   define LSM6DSO_REG_CTRL2_G_ODR_833HZ  (0x07 << 4)
#   define LSM6DSO_REG_CTRL2_G_ODR_1667HZ  (0x08 << 4)
#   define LSM6DSO_REG_CTRL2_G_ODR_3333HZ  (0x09 << 4)
#   define LSM6DSO_REG_CTRL2_G_ODR_6667HZ  (0x0A << 4)
#define LSM6DSO_REG_CTRL3_C    0x12
#   define LSM6DSO_REG_CTRL3_C_SW_RESET   (1 << 0)
#   define LSM6DSO_REG_CTRL3_C_IF_INC     (1 << 2)
#   define LSM6DSO_REG_CTRL3_C_BDU        (1 << 6)
#define LSM6DSO_REG_STATUS_REG 0x1E
#   define LSM6DSO_REG_STATUS_REG_XLDA    (1 << 0)
#   define LSM6DSO_REG_STATUS_REG_GDA     (1 << 1)
#define LSM6DSO_REG_OUTX_L_A   0x28
#define LSM6DSO_REG_OUTX_H_A   0x29
#define LSM6DSO_REG_OUTY_L_A   0x2A
#define LSM6DSO_REG_OUTY_H_A   0x2B
#define LSM6DSO_REG_OUTZ_L_A   0x2C
#define LSM6DSO_REG_OUTZ_H_A   0x2D
#define LSM6DSO_REG_OUT_TEMP_L 0x20
#define LSM6DSO_REG_OUT_TEMP_H 0x21
#define LSM6DSO_REG_OUTX_L_G    0x22
#define LSM6DSO_REG_OUTX_H_G    0x23
#define LSM6DSO_REG_OUTY_L_G    0x24
#define LSM6DSO_REG_OUTY_H_G    0x25
#define LSM6DSO_REG_OUTZ_L_G    0x26
#define LSM6DSO_REG_OUTZ_H_G    0x27
#define LSM6DSO_REG_FIFO_STATUS1    0x3A
#define LSM6DSO_REG_FIFO_STATUS2    0x3B
#   define LSM6DSO_REG_FIFO_STATUS2_DIFF_MASK 0x07
#define LSM6DSO_REG_FIFO_DATA_OUT_TAG 0x78
#   define LSM6DSO_REG_FIFO_DATA_OUT_TAG_MASK    0xF8
#   define LSM6DSO_REG_FIFO_DATA_OUT_TAG_G    (0x01 << 3)
#   define LSM6DSO_REG_FIFO_DATA_OUT_TAG_XL    (0x02 << 3)

AP_InertialSensor_LSM6DSO::AP_InertialSensor_LSM6DSO(AP_InertialSensor &imu,
                                                     AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                     enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _rotation(rotation)
{
}

AP_InertialSensor_Backend *AP_InertialSensor_LSM6DSO::probe(AP_InertialSensor &imu,
                                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                            enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    auto sensor = NEW_NOTHROW AP_InertialSensor_LSM6DSO(imu, std::move(dev), rotation);
    if (!sensor || !sensor->_init_sensor()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}


bool AP_InertialSensor_LSM6DSO::_init_sensor()
{
    return _hardware_init();
}

bool AP_InertialSensor_LSM6DSO::_hardware_init()
{
    WITH_SEMAPHORE(_dev->get_semaphore());

    uint8_t whoami = _register_read(LSM6DSO_REG_WHO_AM_I);
    if (whoami != WHO_AM_I) {
        DEV_PRINTF("LSM6DSO: unexpected acc/gyro WHOAMI 0x%x\n", whoami);
        return false;
    }

    _soft_reset();
    _fifo_init();
    _gyro_init();
    _accel_init();


    hal.scheduler->delay(10);

    return true;
}

void AP_InertialSensor_LSM6DSO::start()
{
    // Register with actual ODR
    if (!_imu.register_gyro(gyro_instance, LSM6DSO_REG_CTRL2_G_ODR, _dev->get_bus_id_devtype(DEVTYPE_INS_LSM6DSO)) ||
        !_imu.register_accel(accel_instance, LSM6DSO_REG_CTRL1_XL_ODR, _dev->get_bus_id_devtype(DEVTYPE_INS_LSM6DSO))) {
        return;
    }

    set_gyro_orientation(gyro_instance, _rotation);
    set_accel_orientation(accel_instance, _rotation);

    _dev->register_periodic_callback(1000,
        FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM6DSO::_poll_data, void));
}

bool AP_InertialSensor_LSM6DSO::update()
{
    // Push to sample queue
    update_gyro(gyro_instance);
    update_accel(accel_instance);

    return true;
}

void AP_InertialSensor_LSM6DSO::_register_write(uint8_t reg, uint8_t val)
{
    _dev->write_register(reg, val);
}

uint8_t AP_InertialSensor_LSM6DSO::_register_read(uint8_t reg)
{
    uint8_t val = 0;
    _dev->read_registers(reg, &val, 1);
    return val;
}

bool AP_InertialSensor_LSM6DSO::_block_read(uint8_t reg, uint8_t *buf, uint32_t len)
{
    return _dev->read_registers(reg, buf, len);
}

void AP_InertialSensor_LSM6DSO::_soft_reset()
{
    _register_write(LSM6DSO_REG_CTRL3_C, LSM6DSO_REG_CTRL3_C_SW_RESET);
    hal.scheduler->delay(100);
}

void AP_InertialSensor_LSM6DSO::_gyro_init()
{
    _register_write(LSM6DSO_REG_CTRL2_G,
                    LSM6DSO_REG_CTRL2_G_ODR_1667HZ |
                    LSM6DSO_REG_CTRL2_G_FS_2000DPS);

    _set_gyro_scale();
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM6DSO::_accel_init()
{
    _register_write(LSM6DSO_REG_CTRL1_XL,
                    LSM6DSO_REG_CTRL1_XL_ODR_1667HZ |
                    LSM6DSO_REG_CTRL1_XL_FS_8G);

    _set_accel_scale();
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM6DSO::_set_gyro_scale()
{
    // Correct scale for 2000DPS range (32768 counts)
    // Convert DPS to rad/s: 2000 * (pi/180) = 2000 * 0.01745329251f
    _gyro_scale = (2000.0f * 0.01745329251f) / 32768.0f;
}

void AP_InertialSensor_LSM6DSO::_set_accel_scale()
{
    // Correct scale for 8G range (32768 counts)
    _accel_scale = (8.0f * GRAVITY_MSS) / 32768.0f;
}

void AP_InertialSensor_LSM6DSO::_fifo_init()
{
    _register_write(LSM6DSO_REG_FIFO_CTRL4, LSM6DSO_REG_FIFO_CTRL4_FMODE_BYPASS);
    hal.scheduler->delay(1);

    _register_write(LSM6DSO_REG_FIFO_CTRL1, LSM6DSO_REG_FIFO_CTRL1_WTM_1);

    _register_write(LSM6DSO_REG_FIFO_CTRL3,
                    LSM6DSO_REG_FIFO_CTRL3_BDR_GY_1667HZ |
                    LSM6DSO_REG_FIFO_CTRL3_BDR_XL_1667HZ);

    // Enable block data update and auto-increment
    _register_write(LSM6DSO_REG_CTRL3_C,
                    LSM6DSO_REG_CTRL3_C_BDU |
                    LSM6DSO_REG_CTRL3_C_IF_INC);

    _register_write(LSM6DSO_REG_FIFO_CTRL4, LSM6DSO_REG_FIFO_CTRL4_FMODE_CON);
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM6DSO::_poll_data()
{
    uint8_t fifo_status[2];

    if (!_block_read(LSM6DSO_REG_FIFO_STATUS1, fifo_status, 2)) {
        return;
    }

    uint16_t unread_data_count = fifo_status[0] | ((fifo_status[1] & LSM6DSO_REG_FIFO_STATUS2_DIFF_MASK) << 8);

    if (unread_data_count > 0) {
        uint16_t samples_to_read = MIN(unread_data_count, FIFO_BATCH_SIZE);

        if (!_block_read(LSM6DSO_REG_FIFO_DATA_OUT_TAG, fifo_buf, samples_to_read * FIFO_SAMPLE_BYTES)) {
            return;
        }

        for (int i = 0; i < samples_to_read; i++) {
            int offset = i * FIFO_SAMPLE_BYTES;

            uint8_t tag = fifo_buf[offset] & LSM6DSO_REG_FIFO_DATA_OUT_TAG_MASK;

            switch (tag) {
            case LSM6DSO_REG_FIFO_DATA_OUT_TAG_G:
                _read_gyro_data(&fifo_buf[offset + 1]);
                break;
            case LSM6DSO_REG_FIFO_DATA_OUT_TAG_XL:
                _read_accel_data(&fifo_buf[offset + 1]);
                break;
            default:
                break;
            }
        }
    }
}

void AP_InertialSensor_LSM6DSO::_read_gyro_data(uint8_t* gyro_data)
{
    Vector3f gyro(
        (int16_t)((gyro_data[1] << 8) | gyro_data[0]),  // X (little-endian)
        (int16_t)((gyro_data[3] << 8) | gyro_data[2]),  // Y
        (int16_t)((gyro_data[5] << 8) | gyro_data[4])); // Z
    gyro *= _gyro_scale;

    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro);
}

void AP_InertialSensor_LSM6DSO::_read_accel_data(uint8_t* accel_data)
{
    Vector3f accel(
        (int16_t)((accel_data[1] << 8) | accel_data[0]),  // X (little-endian)
        (int16_t)((accel_data[3] << 8) | accel_data[2]),  // Y
        (int16_t)((accel_data[5] << 8) | accel_data[4])); // Z
    accel *= _accel_scale;

    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel);
}
