/*
 * SHT3x.h
 *
 *  Created on: Jul 23, 2024
 *      Author: Sovichea
 */

#ifndef SHT3X_H_
#define SHT3X_H_

#include <stdio.h>
#include <stdint.h>

/**
 * Registers addresses.
 */
typedef enum
{
	SHT3X_COMMAND_MEASURE_HIGHREP_STRETCH = 0x2c06,
	SHT3X_COMMAND_CLEAR_STATUS = 0x3041,
	SHT3X_COMMAND_SOFT_RESET = 0x30A2,
	SHT3X_COMMAND_HEATER_ENABLE = 0x306d,
	SHT3X_COMMAND_HEATER_DISABLE = 0x3066,
	SHT3X_COMMAND_READ_STATUS = 0xf32d,
	SHT3X_COMMAND_FETCH_DATA = 0xe000,
	SHT3X_COMMAND_MEASURE_HIGHREP_10HZ = 0x2737,
	SHT3X_COMMAND_MEASURE_LOWREP_10HZ = 0x272a
} sht3x_command_t;

typedef struct
{
  uint8_t temp_h;
  uint8_t temp_l;
  uint8_t temp_crc;
  uint8_t humi_h;
  uint8_t humi_l;
  uint8_t humi_crc;
} SHT3x_data_t;

typedef struct {
	SHT3x_data_t data;
	uint8_t addr;

	/* Callback functions */
	int (*read)(uint8_t addr, uint8_t *buf, uint16_t size, uint16_t timeout, void *arg);
	int (*write)(uint8_t addr, uint8_t *buf, uint16_t size, uint16_t timeout, void *arg);
} SHT3x_handle_t;

int SHT3x_Read(SHT3x_handle_t *SHT3x_handle);
void SHT3x_TurnOn_Heater(SHT3x_handle_t *SHT3x_handle);
void SHT3x_TurnOff_Heater(SHT3x_handle_t *SHT3x_handle);
float SHT3x_GetTemperature(SHT3x_handle_t *SHT3x_handle);
float SHT3x_GetHumidity(SHT3x_handle_t *SHT3x_handle);

#endif /* SHT3X_H_ */
