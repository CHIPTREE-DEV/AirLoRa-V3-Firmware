/*
 * DFRobot_MultiGasSensor.c
 *
 *  Created on: Jul 22, 2024
 *      Author: Sovichea
 */
#include "DFRobot_MultiGasSensor.h"
#include <string.h>
#include <math.h>

#define WRITE_TIMEOUT ( 1000U ) // ms
#define READ_TIMEOUT  ( 1000U ) // ms

static uint8_t checksum(uint8_t* i, uint8_t len)
{
	uint8_t j, tempq = 0;
	i += 1;
	for (j = 0; j < (len - 2); j++)
	{
		tempq += *i;
		i++;
	}
	tempq = (~tempq) + 1;
	return tempq;
}
static void pack(DFRobotMGS_protocol_t *_protocol, uint8_t *pBuf, uint8_t len)
{
	_protocol->head = 0xff;
	_protocol->addr = 0x01;
	memcpy(&_protocol->data, pBuf, len);
	_protocol->check = checksum((uint8_t *)_protocol, 8);
}
static float calculateTemperature(uint16_t temp_ADC)
{
	float Vpd3 = 3 * (float)temp_ADC / 1024.0;
	float Rth = Vpd3 * 10000.0 / (3.0 - Vpd3);
	return 1.0 / ((1.0 / (273.15 + 25.0)) + (1.0 / 3380.13 * log(Rth / 10000.0))) - 273.15;
}

static float applyTemperatureCompensation(DFRobotMGS_handle_t *DFRobotMGS_handle)
{
  DFRobotMGS_gasType gas_type = (DFRobotMGS_gasType) DFRobotMGS_handle->data.gastype;
	float factor = gas_type == CO ? 0.1 : 1.0;

	uint16_t temp_ADC = (DFRobotMGS_handle->data.temp_h << 8) | DFRobotMGS_handle->data.temp_l;
	float Tbeta = calculateTemperature(temp_ADC);

	float gas_concentration = (float)((DFRobotMGS_handle->data.gasconcentration_h << 8)
			| DFRobotMGS_handle->data.gasconcentration_l);

	switch (gas_type)
	{
	case NO2:
		if ((Tbeta > -20) && (Tbeta <= 0)){
				return (gas_concentration * factor / (0.005 * Tbeta + 0.9)) - (-0.0025 * Tbeta + 0.005);
		} else if ((Tbeta > 0) && (Tbeta <= 20)){
				return (gas_concentration * factor / (0.005 * Tbeta + 0.9)) - (0.005 * Tbeta + 0.005);
		} else if ((Tbeta > 20) && (Tbeta <= 40)){
				return (gas_concentration * factor / (0.005 * Tbeta + 0.9)) - (0.0025 * Tbeta + 0.1);
		}
		break;

	case O3:
		if ((Tbeta > -20) && (Tbeta <= 0)){
				return (gas_concentration * factor / (0.015 * Tbeta + 1.1)) - 0.05;
		} else if ((Tbeta > 0) && (Tbeta <= 20)){
				return (gas_concentration * factor / 1.1) - (0.01 * Tbeta);
		} else if ((Tbeta > 20) && (Tbeta <= 40)){
				return (gas_concentration * factor / 1.1) - (-0.005 * Tbeta + 0.3);
		}
		break;

	case CO:
		if ((Tbeta > -20) && (Tbeta <= 20)){
				return (gas_concentration * factor / (0.005 * Tbeta + 0.9));
		} else if ((Tbeta > 20) && (Tbeta <= 40)){
				return (gas_concentration * factor / (0.005 * Tbeta + 0.9)) - (0.3 * Tbeta - 6);
		}
		break;

	default:
		break;
	}

	return 0;
}
void DFRobotMGS_Read(DFRobotMGS_handle_t *DFRobotMGS_handle)
{
	DFRobotMGS_protocol_t _protocol;
	uint8_t buf[6] = {0};
	buf[0] = CMD_GET_ALL_DATA;
	pack(&_protocol, buf, sizeof(buf));
	uint8_t addr = DFRobotMGS_handle->addr;
	DFRobotMGS_handle->write(addr, (uint8_t*)&_protocol, sizeof(DFRobotMGS_protocol_t), WRITE_TIMEOUT, NULL);
	DFRobotMGS_handle->read(addr, (uint8_t*)&DFRobotMGS_handle->data, sizeof(DFRobotMGS_data_t), READ_TIMEOUT, NULL);
}

float DFRobotMGS_Temp(DFRobotMGS_handle_t *DFRobotMGS_handle){
	uint16_t Temp = (DFRobotMGS_handle->data.temp_h << 8) | DFRobotMGS_handle->data.temp_l;
	float Tbeta = calculateTemperature(Temp);

	return Tbeta;

}
float DFRobotMGS_GetNO2(DFRobotMGS_handle_t *DFRobotMGS_handle)
{
	uint16_t NO2 = (DFRobotMGS_handle->data.gasconcentration_h << 8) | DFRobotMGS_handle->data.gasconcentration_l;
	if (!DFRobotMGS_handle->temp_compensation)
		return (float)NO2;

	float NO2_float = applyTemperatureCompensation(DFRobotMGS_handle);
	return NO2_float;
}

float DFRobotMGS_GetO3(DFRobotMGS_handle_t *DFRobotMGS_handle)
{
	uint16_t O3 = (DFRobotMGS_handle->data.gasconcentration_h << 8) | DFRobotMGS_handle->data.gasconcentration_l;
	if (!DFRobotMGS_handle->temp_compensation)
		return (float)O3;

	float O3_float = applyTemperatureCompensation(DFRobotMGS_handle);
	return O3_float;
}

float DFRobotMGS_GetCO(DFRobotMGS_handle_t *DFRobotMGS_handle)
{
	uint16_t CO = (DFRobotMGS_handle->data.gasconcentration_h << 8) | DFRobotMGS_handle->data.gasconcentration_l;
	if (!DFRobotMGS_handle->temp_compensation)
		return (float)CO;

	float CO_float = applyTemperatureCompensation(DFRobotMGS_handle);
	return CO_float;
}
