/*
 * DFRobotMGS_MultiGasSensor.h
 *
 *  Created on: Jul 22, 2024
 *      Author: Sovichea
 */

#ifndef DFRobotMGS_MULTIGASSENSOR_H_
#define DFRobotMGS_MULTIGASSENSOR_H_

#include <stdint.h>

#define CMD_CHANGE_GET_METHOD          0x78
#define CMD_GET_GAS_CONCENTRATION      0x86
#define CMD_GET_TEMP                   0x87
#define CMD_GET_ALL_DATA               0x88
#define CMD_SET_THRESHOLD_ALARMS       0x89
#define CMD_I2C_AVAILABLE              0x90
#define CMD_SENSOR_VOLTAGE             0x91
#define CMD_CHANGE_I2C_ADDR            0x92

/**
 * @struct DFRobotMGS_protocol_t
 * @brief Data protocol package for communication
 */
typedef struct
{
  uint8_t head;
  uint8_t addr;
  uint8_t data[6];
  uint8_t check;
} DFRobotMGS_protocol_t;

/**
 * @struct DFRobotMGS_data_t
 * @brief The struct used when getting all the data
 * @note
 * @n -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * @n |  byte0          | byte1    |          byte2             |        byte3                |  byte4   |  byte5         |  byte6                 |   byte7               |  byte8
 * @n -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * @n |  Protocol Head  | Command  | Gas Concentrate High 8-bit | Gas Concentration Low 8-bit | Gas Type | Decimal Digits | Temperature High 8-bit | Temperature Low 8-bit |  CRC
 * @n -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */
typedef struct
{
  uint8_t head;
  uint8_t cmd;
  uint8_t gasconcentration_h;
  uint8_t gasconcentration_l;
  uint8_t gastype;
  uint8_t gasconcentration_decimals;
  uint8_t temp_h;
  uint8_t temp_l;
  uint8_t check;
} DFRobotMGS_data_t;

/**
 * @enum DFRobotMGS_GasType
 * @brief Gas Type
 */
typedef enum
{
	O2   = 0x05,
	CO   = 0x04,
	H2S  = 0x03,
	NO2  = 0x2C,
	O3   = 0x2A,
	CL2  = 0x31,
	NH3  = 0x02,
	H2   = 0x06,
	HCL  = 0X2E,
	SO2  = 0X2B,
	HF   = 0x33,
	_PH3 = 0x45
} DFRobotMGS_gasType;

typedef struct {
	DFRobotMGS_data_t data;
	uint8_t temp_compensation;
	uint8_t addr;

	/* Callback functions */
	int (*read)(uint8_t addr, uint8_t *buf, uint16_t size, uint16_t timeout, void *arg);
	int (*write)(uint8_t addr, uint8_t *buf, uint16_t size, uint16_t timeout, void *arg);
} DFRobotMGS_handle_t;

void DFRobotMGS_Read(DFRobotMGS_handle_t *DFRobotMGS_handle);
float DFRobotMGS_GetNO2(DFRobotMGS_handle_t *DFRobotMGS_handle);
float DFRobotMGS_GetO3(DFRobotMGS_handle_t *DFRobotMGS_handle);
float DFRobotMGS_GetCO(DFRobotMGS_handle_t *DFRobotMGS_handle);
float DFRobotMGS_Temp(DFRobotMGS_handle_t *DFRobotMGS_handle);


#endif /* D_DFRobotMGS_MULTIGASSENSOR_H_ */
