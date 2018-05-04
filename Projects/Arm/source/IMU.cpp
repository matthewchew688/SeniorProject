#include "IMU.h"

esp_err_t I2C_gimbal_write(i2c_port_t i2c_num, uint8_t address, uint8_t data)
{
    esp_err_t ret;
    i2c_cmd_handle_t g_write = i2c_cmd_link_create();
    ret = i2c_master_start(g_write);
    ret = i2c_master_write_byte(g_write, LIS3DH_SA0_1_WRITE, ACK_EN);
    ret = i2c_master_write_byte(g_write, address, ACK_EN);
    ret = i2c_master_write_byte(g_write, data, ACK_EN);
    ret = i2c_master_stop(g_write);
    ret = i2c_master_cmd_begin(i2c_num, g_write, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(g_write);
    return ret;
}

esp_err_t I2C_gimbal_write_multi(i2c_port_t i2c_num, uint8_t address, uint8_t *data, uint8_t size)
{
    esp_err_t ret;
    i2c_cmd_handle_t g_write = i2c_cmd_link_create();
    ret = i2c_master_start(g_write);
    ret = i2c_master_write_byte(g_write, LIS3DH_SA0_1_WRITE, ACK_EN);
    ret = i2c_master_write_byte(g_write, address, ACK_EN);
    ret = i2c_master_write(g_write, data, size - 1, ACK_EN);
    ret = i2c_master_stop(g_write);
    ret = i2c_master_cmd_begin(i2c_num, g_write, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(g_write);
    return ret;
}

esp_err_t I2C_gimbal_read(i2c_port_t i2c_num, uint8_t address, uint8_t *data)
{
    esp_err_t ret;
    i2c_cmd_handle_t g_read = i2c_cmd_link_create();
    ret = i2c_master_start(g_read);
    ret = i2c_master_write_byte(g_read, LIS3DH_SA0_1_WRITE, ACK_EN);
    ret = i2c_master_write_byte(g_read, address, ACK_EN);
    ret = i2c_master_read_byte(g_read, data, ACK_VAL);
    ret = i2c_master_stop(g_read);
    ret = i2c_master_cmd_begin(i2c_num, g_read, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(g_read);
    return ret;
}

esp_err_t I2C_gimbal_read_multi(i2c_port_t i2c_num, uint8_t address, uint8_t *data, uint8_t size)
{
    esp_err_t ret;
    i2c_cmd_handle_t g_read = i2c_cmd_link_create();
    ret = i2c_master_start(g_read);
    ret = i2c_master_write_byte(g_read, LIS3DH_SA0_1_WRITE, ACK_EN);
    ret = i2c_master_write_byte(g_read, address, ACK_EN);
    ret = i2c_master_read(g_read, data, size - 1, ACK_VAL);
    ret = i2c_master_stop(g_read);
    ret = i2c_master_cmd_begin(i2c_num, g_read, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(g_read);
    return ret;
}

esp_err_t I2C_accel_opr_setup(i2c_port_t i2c_num, uint8_t data)
{
    esp_err_t ret;
 
    i2c_cmd_handle_t a_write = i2c_cmd_link_create();
    i2c_master_start(a_write);
    ret = i2c_master_write_byte(a_write, (BNO055_ADDR_COM3_HIGH << 1) | WRITE_BIT, ACK_EN);
    ret = i2c_master_write_byte(a_write, BNO055_OPR_MODE_ADDR, ACK_EN);
    ret = i2c_master_write_byte(a_write, data, ACK_EN);
    ret = i2c_master_stop(a_write);
    ret = i2c_master_cmd_begin(i2c_num, a_write, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(a_write);
    return ret;
}

esp_err_t I2C_accel_read_multi(i2c_port_t i2c_num, uint8_t address, uint8_t *data, uint8_t size)
{
    esp_err_t ret;
 
    i2c_cmd_handle_t a_read = i2c_cmd_link_create();
    i2c_master_start(a_read);
    ret = i2c_master_write_byte(a_read, BNO055_WRITE_HIGH_ADDR, ACK_EN);
    ret = i2c_master_write_byte(a_read, address, ACK_EN);
    ret = i2c_master_stop(a_read);
    ret = i2c_master_cmd_begin(i2c_num, a_read, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(a_read);
 
    // printf("\n");
 
    a_read = i2c_cmd_link_create();
    i2c_master_start(a_read);
    ret = i2c_master_write_byte(a_read, BNO055_READ_HIGH_ADDR, ACK_EN);
    ret = i2c_master_read_byte(a_read, data, ACK_VAL);
    ret = i2c_master_read_byte(a_read, (data + 1), NACK_VAL);
    ret = i2c_master_stop(a_read);
    ret = i2c_master_cmd_begin(i2c_num, a_read, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(a_read);
    return ret;
}
