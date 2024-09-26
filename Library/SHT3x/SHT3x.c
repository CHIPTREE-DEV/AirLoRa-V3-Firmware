/*
 * SHT3x.c
 *
 *  Created on: Jul 23, 2024
 *      Author: Sovichea
 */

#include "SHT3x.h"

#define WRITE_TIMEOUT ( 1000U )
#define READ_TIMEOUT  ( 1000U )

static uint8_t calculate_crc(const uint8_t *data, uint16_t length)
{
	uint8_t crc = 0xff;
	for (size_t i = 0; i < length; i++) {
		crc ^= data[i];
		for (size_t j = 0; j < 8; j++) {
			if ((crc & 0x80u) != 0) {
				crc = (uint8_t)((uint8_t)(crc << 1u) ^ 0x31u);
			} else {
				crc <<= 1u;
			}
		}
	}
	return crc;
}

int SHT3x_Read(SHT3x_handle_t *SHT3x_handle)
{
	int retcode;
	uint16_t cmd = SHT3X_COMMAND_MEASURE_HIGHREP_STRETCH;
	uint8_t cmd_buf[2] = {(uint8_t)(cmd >> 8), (uint8_t)cmd};
	retcode = SHT3x_handle->write(SHT3x_handle->addr, cmd_buf, 2, WRITE_TIMEOUT, NULL);
	if (retcode == 0) return 0;

	retcode = SHT3x_handle->read(SHT3x_handle->addr, (uint8_t*)&SHT3x_handle->data, sizeof(SHT3x_data_t), READ_TIMEOUT, NULL);
	if (retcode == 0) return 0;

	uint8_t temperature_crc = calculate_crc((uint8_t*)&SHT3x_handle->data.temp_h, 2);
	uint8_t humidity_crc = calculate_crc((uint8_t*)&SHT3x_handle->data.humi_h, 2);
	if (temperature_crc != SHT3x_handle->data.temp_crc ||
			humidity_crc != SHT3x_handle->data.humi_crc) {
		return 0;
	}

	return 1;
}

float SHT3x_GetTemperature(SHT3x_handle_t *SHT3x_handle)
{
	uint16_t temperature_raw = (SHT3x_handle->data.temp_h << 8) | SHT3x_handle->data.temp_l;
	float temperature = -45.0f + 175.0f * (float)temperature_raw / 65535.0f;
	return temperature;
}

float SHT3x_GetHumidity(SHT3x_handle_t *SHT3x_handle)
{
	uint16_t humidity_raw = (SHT3x_handle->data.humi_h << 8) | SHT3x_handle->data.humi_l;
	float humidity = 100.0f * (float)humidity_raw / 65535.0f;
	return humidity;
}

void SHT3x_TurnOn_Heater(SHT3x_handle_t *SHT3x_handle){

	uint16_t cmd = SHT3X_COMMAND_HEATER_ENABLE;

	uint8_t cmd_buf[2] = {(uint8_t)(cmd >> 8), (uint8_t)cmd};

	SHT3x_handle->write(SHT3x_handle->addr, cmd_buf, 2, WRITE_TIMEOUT, NULL);

}

void SHT3x_TurnOff_Heater(SHT3x_handle_t *SHT3x_handle){

	uint16_t cmd = SHT3X_COMMAND_HEATER_DISABLE;

	uint8_t cmd_buf[2] = {(uint8_t)(cmd >> 8), (uint8_t)cmd};

	SHT3x_handle->write(SHT3x_handle->addr, cmd_buf, 2, WRITE_TIMEOUT, NULL);

}




