/*
 * DF_ROBOT_GAS.h
 *
 *  Created on: May 23, 2024
 *      Author: macbookpro
 */

#ifndef INC_DF_ROBOT_GAS_H_
#define INC_DF_ROBOT_GAS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"

extern I2C_HandleTypeDef hi2c1;  // Assuming hi2c1 is declared somewhere globally

// Define sensor I2C addresses
#define O3_ADDR (0x74 << 1)  // ADDRESS_O3: 0x74, A0=0, A1=0
#define NO2_ADDR (0x75 << 1) // ADDRESS_NO2: 0x75, A0=0, A1=1
#define CO_ADDR (0x76 << 1)  // ADDRESS_CO: 0x76, A0=1, A1=0

// Function prototypes

void readSensorData(uint16_t addr, uint8_t *ppm, float multiplier);
uint8_t extractConcentration(uint8_t *data, float multiplier);

#ifdef __cplusplus
}
#endif

#endif /* INC_DF_ROBOT_GAS_H_ */
