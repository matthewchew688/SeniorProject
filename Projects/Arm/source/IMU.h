#ifndef IMU_H_
#define IMU_H_

#include "driver/i2c.h"
#include "Arm_Defines.h"
#include "common_defines.h"

esp_err_t I2C_accel_opr_setup(i2c_port_t i2c_num, uint8_t data);

esp_err_t I2C_accel_read_multi(i2c_port_t i2c_num, uint8_t address, uint8_t *data, uint8_t size);

esp_err_t I2C_gimbal_write(i2c_port_t i2c_num, uint8_t address, uint8_t data);

esp_err_t I2C_gimbal_write_multi(i2c_port_t i2c_num, uint8_t address, uint8_t data, uint8_t size);

esp_err_t I2C_gimbal_read(i2c_port_t i2c_num, uint8_t address, uint8_t *data);

esp_err_t I2C_gimbal_read_multi(i2c_port_t i2c_num, uint8_t address, uint8_t *data, uint8_t size);

#endif