/*
 *	@File: PMS5003_HAL_STM32.h
 *
 *	@Created on: May, 05, 2020
 *  @Author: AARONEE - khoahuynh
 * 	@Email: khoahuynh.ld@gmail.com
 * 	@Github: https://github.com/aaronee

 *
 *	@Introduction: This file belongs to PMS5003_HAL_STM32 library
 *
 *
 *	Copyright (C) 2020 - Khoa Huynh
 *  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
 *	of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.
 *
 *	This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
 *	or indirectly by this software, read more about this on the GNU General Public License.
 */

#ifndef PMS5003_H_
#define PMS5003_H_

#include <stdio.h>
#include <stdint.h>

/*!<@PMS_MODE_define>*/
#define PMS_MODE_ACTIVE 1	//Mode active: sensor send data continuously (maximum ~800ms interval)
							//interval time can be varied depending on the change rate of collected datas
#define PMS_MODE_PASSIVE 0	//Mode passive: sensor send data when receiving a request
//-----------------------

typedef struct {
	uint16_t PM1_0_factory;		 // concentration unit * μg/m3 - under factory environment
	uint16_t PM2_5_factory;
	uint16_t PM10_factory;
	uint16_t PM1_0_atmospheric; // concentration unit * μg/m3 - under atmospheric environment
	uint16_t PM2_5_atmospheric;
	uint16_t PM10_atmospheric;
	uint16_t density_0_3um; 	// number of particles with diameter beyond X(um) in 0.1L of air
	uint16_t density_0_5um;
	uint16_t density_1_0um;
	uint16_t density_2_5um;
	uint16_t density_5_0um;
	uint16_t density_10um;
} PMS_data_t;

typedef struct {
	uint8_t mode;
	PMS_data_t data;

	/* Callback functions */
	int (*read)(uint8_t *buf, uint16_t size, uint16_t timeout, void *arg);
	int (*write)(uint8_t *buf, uint16_t size, uint16_t timeout, void *arg);
}PMS_handle_t;


int PMS_Init(PMS_handle_t *PMS_handle);
int PMS_swmode(PMS_handle_t *PMS_handle, uint8_t mode);
int PMS_sleep(PMS_handle_t *PMS_handle);
int PMS_wakeup(PMS_handle_t *PMS_handle);
int PMS_read(PMS_handle_t *PMS_handle);

#endif /* PMS5003_H_ */
