/**
 * i2c_platform_esp.h
 * 
 * I2C device interface for the 
 * Espressif Internet-of-Things (IoT) Development Framework ESP-IDF
 *
 * I2C_Master is the global I2C interface shared by all devices
 * 
 * (c) 2021 by David Asher
 * https://github.com/david-asher
 * https://www.linkedin.com/in/davidasher/
 * This code is licensed under MIT license, see LICENSE.txt for details
 */

#ifndef _I2C_PLATFORM_ESP_H_
#define _I2C_PLATFORM_ESP_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define I2C_DEFAULT_PORT    (I2C_NUM_0)
#define I2C_DEFAULT_SDA     (GPIO_NUM_21)
#define I2C_DEFAULT_SCL     (GPIO_NUM_22)
#define I2C_DEFAULT_FREQ    (400000)

#define I2C_READ            (I2C_MASTER_READ)
#define I2C_WRITE           (I2C_MASTER_WRITE)

#define I2C_NO_DEVICE       (0xFF)

#define ACK_CHECK_EN        true

void i2c_scan(i2c_master_bus_handle_t bus_handle);

#ifdef __cplusplus
}
#endif

#endif // _I2C_PLATFORM_ESP_H_
