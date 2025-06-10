/**
 * vl53l1_platform.c
 * 
 * Platform-specific interface functions for the ST VL53L1 family of
 * time-of-flight laser ranging sensors, specific implementation for the
 * Espressif Internet-of-Things (IoT) Development Framework ESP-IDF
 *
 * (c) 2021 by David Asher
 * https://github.com/david-asher
 * https://www.linkedin.com/in/davidasher/
 * This code is licensed under MIT license, see LICENSE.txt for details
 *
 * Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
 * 
 * License terms: BSD 3-clause "New" or "Revised" License. 
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this 
 * list of conditions and the following disclaimer. 
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution. 
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors 
 * may be used to endorse or promote products derived from this software 
 * without specific prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 * 
 */


#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "VL53L1X_api.h"

static const uint8_t status_rtn[24] = { 
	255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
	255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
	255, 255, 11, 12
};

/**
 * vl53l1 platform-specific implementation, I2C read-write device functions
 */

VL53L1X_ERROR esp_to_vl53l1x_error( esp_err_t esp_code )
{
    switch( esp_code ) {
    case ESP_OK:                    return VL53L1_ERROR_NONE;
    case ESP_FAIL:                  return VL53L1_ERROR_CONTROL_INTERFACE;
    case ESP_ERR_NO_MEM:            return VL53L1_ERROR_BUFFER_TOO_SMALL;
    case ESP_ERR_INVALID_ARG:       return VL53L1_ERROR_INVALID_PARAMS;
    case ESP_ERR_INVALID_STATE:     return VL53L1_ERROR_INVALID_COMMAND;
    case ESP_ERR_INVALID_SIZE:      return VL53L1_ERROR_INVALID_COMMAND;
    case ESP_ERR_NOT_FOUND:         return VL53L1_ERROR_NOT_SUPPORTED;
    case ESP_ERR_NOT_SUPPORTED:     return VL53L1_ERROR_NOT_SUPPORTED;
    case ESP_ERR_TIMEOUT:           return VL53L1_ERROR_TIME_OUT;
    case ESP_ERR_INVALID_RESPONSE:  return VL53L1_ERROR_CONTROL_INTERFACE;
    case ESP_ERR_INVALID_CRC:       return VL53L1_ERROR_CONTROL_INTERFACE;
    case ESP_ERR_INVALID_VERSION:   return VL53L1_ERROR_CONTROL_INTERFACE;
    case ESP_ERR_INVALID_MAC:       return VL53L1_ERROR_CONTROL_INTERFACE;
    case ESP_ERR_WIFI_BASE:         return VL53L1_ERROR_CONTROL_INTERFACE;
    case ESP_ERR_MESH_BASE:         return VL53L1_ERROR_CONTROL_INTERFACE;
    case ESP_ERR_FLASH_BASE:        return VL53L1_ERROR_CONTROL_INTERFACE;
    default:                        return VL53L1_ERROR_UNDEFINED;
    }
}

VL53L1X_ERROR VL53L1_WriteMulti(i2c_master_dev_handle_t dev_handle, uint16_t index,
    uint8_t *pdata, uint32_t count)
{
    esp_err_t err;
    uint8_t write_buf[2 + count];

    // Insert index (register address) in big-endian format
    write_buf[0] = index >> 8;
    write_buf[1] = index & 0xFF;

    // Copy data payload
    memcpy(&write_buf[2], pdata, count);

    err = i2c_master_transmit(dev_handle, write_buf, 2 + count, 1000);

    return esp_to_vl53l1x_error(err);
}

VL53L1X_ERROR VL53L1_ReadMulti(i2c_master_dev_handle_t dev_handle, uint16_t index,
    uint8_t *pdata, uint32_t count)
{
    esp_err_t err;
    uint8_t index_buf[2];

    index_buf[0] = index >> 8;     // MSB
    index_buf[1] = index & 0xFF;   // LSB

    // Perform combined write (index) + read (pdata)
    err = i2c_master_transmit_receive(dev_handle, index_buf, sizeof(index_buf), 
        pdata, count, 1000);

    return esp_to_vl53l1x_error(err);
}

VL53L1X_ERROR VL53L1_WrByte(i2c_master_dev_handle_t dev_handle, uint16_t index,
    uint8_t data) 
{
    int  status;
    uint8_t write_data = data;
    status = VL53L1_WriteMulti(dev_handle, index, &write_data, 1);
    return status;
}

VL53L1X_ERROR VL53L1_WrWord(i2c_master_dev_handle_t dev_handle, uint16_t index,
    uint16_t data) 
{
    int  status;
    uint8_t buffer[2];
    buffer[0] = data >> 8;
    buffer[1] = data & 0x00FF;
    status = VL53L1_WriteMulti(dev_handle, index, (uint8_t *)buffer, 2);
    return status;
}

VL53L1X_ERROR VL53L1_WrDWord(i2c_master_dev_handle_t dev_handle, uint16_t index,
    uint32_t data) 
{
    int  status;
    uint8_t buffer[4];
    buffer[0] = (data >> 24) & 0xFF;
    buffer[1] = (data >> 16) & 0xFF;
    buffer[2] = (data >>  8) & 0xFF;
    buffer[3] = (data >>  0) & 0xFF;
    status = VL53L1_WriteMulti(dev_handle, index, (uint8_t *)buffer, 4);
    return status;
}

VL53L1X_ERROR VL53L1_UpdateByte(i2c_master_dev_handle_t dev_handle, uint16_t index,
    uint8_t AndData, uint8_t OrData) 
{
    int  status;
    uint8_t buffer = 0;

    /* read data direct onto buffer */
    status = VL53L1_ReadMulti(dev_handle, index, &buffer, 1);
    if (status) return status;
    buffer = (buffer & AndData) | OrData;
    status = VL53L1_WriteMulti(dev_handle, index, &buffer, (uint16_t)1);
    return status;
}

VL53L1X_ERROR VL53L1_RdByte(i2c_master_dev_handle_t dev_handle, uint16_t index,
    uint8_t *data) 
{
    int  status;
    status = VL53L1_ReadMulti(dev_handle, index, data, 1);
    return status ? -1 : 0;
}

VL53L1X_ERROR VL53L1_RdWord(i2c_master_dev_handle_t dev_handle, uint16_t index,
    uint16_t *data) 
{
    int  status;
    uint8_t buffer[2] = {0, 0};
    status = VL53L1_ReadMulti(dev_handle, index, buffer, 2);
    if ( status ) return status;
    *data = (buffer[0] << 8) + buffer[1];
    return status;
}

VL53L1X_ERROR VL53L1_RdDWord(i2c_master_dev_handle_t dev_handle, uint16_t index,
    uint32_t *data) 
{
    int status;
    uint8_t buffer[4] = {0, 0, 0, 0};
    status = VL53L1_ReadMulti(dev_handle, index, buffer, 4);
    if ( status ) return status;
    *data = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];
    return status;
}

/**
 * vl53l1 platform-specific implementation, O/S timing functions
 */

VL53L1X_ERROR VL53L1_GetTickCount( uint32_t *ptick_count_ms )
{
    // note: _ms means microseconds, not milliseconds
    *ptick_count_ms = esp_timer_get_time();
	return VL53L1_ERROR_NONE;
}

VL53L1X_ERROR VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
    *ptimer_freq_hz = I2C_DEFAULT_FREQ;
	return VL53L1_ERROR_NONE;
}

// other useful device functions

VL53L1X_ERROR VL53L1X_SetFastI2C(i2c_master_dev_handle_t dev_handle)
{
    return VL53L1_WrByte(dev_handle, VL53L1_PAD_I2C_HV__CONFIG, 0x14 );
}

/**
 * 	fields: \n
 *		- [1:0] = scheduler_mode
 *		- [3:2] = readout_mode
 *		-   [4] = mode_range__single_shot
 *		-   [5] = mode_range__back_to_back
 *		-   [6] = mode_range__timed
 *		-   [7] = mode_range__abort 
 */
VL53L1X_ERROR VL53L1X_SetRangingMode(i2c_master_dev_handle_t dev_handle,
    uint8_t set_ranging_mode)
{
    uint8_t mode_start;
    VL53L1_RdByte(dev_handle, VL53L1_SYSTEM__MODE_START, &mode_start );
    mode_start = ( mode_start & 0x0F ) | set_ranging_mode;
    return VL53L1_WrByte(dev_handle, VL53L1_SYSTEM__MODE_START, mode_start );
}

VL53L1X_ERROR VL53L1X_SystemStatus(i2c_master_dev_handle_t dev_handle, uint8_t *state)
{
	return VL53L1_RdByte(dev_handle, VL53L1_FIRMWARE__SYSTEM_STATUS, state);
}

char *VL53L1X_SystemStatusString(i2c_master_dev_handle_t dev_handle)
{
    uint8_t system_status;
	VL53L1_RdByte(dev_handle, VL53L1_FIRMWARE__SYSTEM_STATUS, &system_status);
    switch( system_status ) {
    case VL53L1_STATE_POWERDOWN:        return "VL53L1_STATE_POWERDOWN";
    case VL53L1_STATE_WAIT_STATICINIT:  return "VL53L1_STATE_WAIT_STATICINIT";
    case VL53L1_STATE_STANDBY:          return "VL53L1_STATE_STANDBY";
    case VL53L1_STATE_IDLE:             return "VL53L1_STATE_IDLE";
    case VL53L1_STATE_RUNNING:          return "VL53L1_STATE_RUNNING";
    case VL53L1_STATE_RESET:            return "VL53L1_STATE_RESET";
    case VL53L1_STATE_UNKNOWN:          return "VL53L1_STATE_UNKNOWN";
    case VL53L1_STATE_ERROR:            return "VL53L1_STATE_ERROR";
    default:                            return "VL53L1_STATE_UNKNOWN";
    }
}

VL53L1X_ERROR VL53L1X_GetContinuousMeasurement(i2c_master_dev_handle_t dev_handle,
    uint8_t *rangeStatus, uint16_t *distanceMM)
{
	VL53L1X_ERROR status;

	// VL53L1X_GetRangeStatus(dev, &RangeStatus)
	uint8_t RgSt;
	status = VL53L1_RdByte(dev_handle, VL53L1_RESULT__RANGE_STATUS, &RgSt);
	*rangeStatus = (RgSt < 24) ? status_rtn[RgSt] : RgSt & 0x1F;

	//	VL53L1X_GetDistance(dev, &Distance)
	VL53L1_RdWord(dev_handle, VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, distanceMM);

    //  VL53L1X_ClearInterrupt(dev)
	VL53L1_WrByte(dev_handle, SYSTEM__INTERRUPT_CLEAR, 0x01);

	return status;
}

VL53L1X_ERROR VL53L1X_GetAndRestartMeasurement(i2c_master_dev_handle_t dev_handle,
    uint8_t *rangeStatus, uint16_t *distanceMM)
{
	VL53L1X_ERROR status;

	// VL53L1X_GetRangeStatus(dev, &RangeStatus)
	uint8_t RgSt;
	status = VL53L1_RdByte(dev_handle, VL53L1_RESULT__RANGE_STATUS, &RgSt);
	*rangeStatus = (RgSt < 24) ? status_rtn[RgSt] : RgSt & 0x1F;

	//	VL53L1X_GetDistance(dev, &Distance)
	VL53L1_RdWord(dev_handle, VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, distanceMM);

    //  VL53L1X_StartRanging(dev)
    VL53L1_WrByte(dev_handle, SYSTEM__MODE_START, 0x40);

	return status;
}
