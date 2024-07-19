/*
 * DF_ROBOT_GAS.c
 *
 *  Created on: May 23, 2024
 *      Author: macbookpro
 */
#include "DF_ROBOT_GAS.h"

// Command to read concentration values
uint8_t Read_concentration_cmd[9] = {0xFF, 0x01, 0x86, 0, 0, 0, 0, 0, 0x79};

//checksum
static uint8_t Checksum (uint8_t* i,uint8_t ln)
{
	uint8_t j;
	uint8_t tempq = 0;
	i+=1;
	for (j=0;j<(ln-2);j++)
	{
		tempq +=*i;
		i++;
	}
	tempq = (~tempq)+1;
	return (tempq);
}

// Function to extract concentration from sensor data and return it as uint8_t
uint8_t extractConcentration(uint8_t *data, float multiplier) {
    int high_byte = data[2];
    int low_byte = data[3];
    int concentration = ((high_byte << 8) | low_byte) * multiplier;

    // Clamp the concentration to fit into uint8_t range
    if (concentration > 255) {
        concentration = 255;
    } else if (concentration < 0) {
        concentration = 0;
    }

    return (uint8_t)concentration;
}

// Function to read sensor data
void readSensorData(uint16_t addr, uint8_t *ppm, float multiplier) {
    uint8_t sensorData[9];

    if (HAL_I2C_Master_Transmit(&hi2c1, addr, Read_concentration_cmd, sizeof(Read_concentration_cmd), 1000) == HAL_OK) {
        HAL_Delay(10); // Short delay before receiving to give sensor time to prepare data

        if (HAL_I2C_Master_Receive(&hi2c1, addr, sensorData, sizeof(sensorData), 1000) == HAL_OK) {
            uint8_t calculate_checksum = Checksum(sensorData, sizeof(sensorData));

            if (sensorData[0] == 0xFF && sensorData[1] == 0x86 && calculate_checksum == sensorData[8]) {
                *ppm = extractConcentration(sensorData, multiplier);
            }
        }
    }
}
