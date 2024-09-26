
/*
 *	@File: PMS5003_HAL_STM32.c
 *
 *	@Created on: May, 05, 2020
 *  @Author: AARONEE - khoahuynh
 * 	@Email: khoahuynh.ld@gmail.com
 * 	@Github: https://github.com/aaronee
 *
 *
 *	@Introduction: This file belongs to PMS5003_HAL_STM32 library
 *				> This is the library writen for air sensor PMS5003
 *				> This library is based on STM32 HAL library. You must use STM32 HAL library to use this library
 *	@MainFunction:
 *				> Initializing air sensor with selected mode
 *				> Sleep sensor
 *				> Wake-up sensor
 *				> Perform verify checksum
 *				> Read data from sensor
 *
 *
 *	Copyright (C) 2020 - Khoa Huynh
 *  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
 *	of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.
 *
 *	This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
 *	or indirectly by this software, read more about this on the GNU General Public License.
 */

#include "PMS5003.h"

#define WRITE_TIMEOUT ( 100U ) // ms
#define READ_TIMEOUT  ( 100U ) // ms

// Host Protocol
// StartByte1 - StartByte2 - COMMAND - DATA1 - DATA2 - VerifyByte1 - VerifyByte2
//  (0x42)	  -   (0x4d)   -  (CMD)  - DATAH - DATAL -   (LRCH)    -   (LRCL)
// Command definition	   -   0XE2  - 	 X	 -  X	 : read in passive mode

//						   -   0XE1  - 	 X	 -  0x00 : change to passive mode
//						   -   0XE1  - 	 X	 -  0x01 : change to active  mode

//						   -   0XE4  - 	 X	 -  0x00 : change to sleep   mode
//						   -   0XE4  - 	 X	 -  0x01 : change to wake up mode

// DATAH = DATA HIGH is DATA1
// DATAL = DATA LOW is DATA2

// LRCH = LRC HIGH
// LRCL = LRC LOW

uint8_t read_passive[7] = {0x42,0x4D,0xe2,0xFF,0xFF,0x03,0x6f};	 // LRC = sum all except verify bytes

uint8_t mode_passive[7] = {0x42,0x4D,0xe1,0xFF,0x00,0x02,0x6f};
uint8_t passive_ack[8]  = {0x42,0x4D,0x00,0x04,0xE1,0x00,0x01,0x74};// acknowledgement from sensor

uint8_t mode_active[7]  = {0x42,0x4D,0xe1,0xFF,0x01,0x02,0x70};
uint8_t active_ack[8]   = {0x42,0x4D,0x00,0x04,0xE1,0x01,0x01,0x75};

uint8_t cmd_sleep[7]    = {0x42,0x4D,0xe4,0xFF,0x00,0x02,0x72};
uint8_t sleep_ack[8]	= {0x42,0x4D,0x00,0x04,0xE4,0x00,0x01,0x77};// acknowledgement from sensor

uint8_t cmd_wakeup[7]   = {0x42,0x4D,0xe4,0xFF,0x01,0x02,0x73};

//PMS5003 transport protocol-Active Mode send back 32bytes data
uint8_t _sleepflag;

uint16_t _framelen;
uint16_t _reserved;
uint16_t _checkcode;

static int _PMS_checksum(uint8_t buf[],uint8_t element);
static int _PMS_checkarray(uint8_t rxbuf[], uint8_t ack[], uint8_t element);

//////////////////////////////////////////////////////////////////////////////////
//
//					Functions Begin
//
//////////////////////////////////////////////////////////////////////////////////

/* PMS_Init
 * @param PMS_handle_t struct
 * @retval: status
 *
 * Use to initialize the sensor, user needs to create PMS_handle_t structure
 * and supply handle uart & select mode for sensor
 *
 * 		PMS_handle->PMS_huart = huart; // to communicate to sensor
 * 		PMS_handle->mode; 		   // look up in @mode_define)
 *
 * After initialized, its recommended to wait at least 30 seconds to read any data
 * */
int PMS_Init(PMS_handle_t *PMS_handle)
{
	int initstatus    = 1;
	uint8_t mode_temp_holder = PMS_handle->mode;

//	initstatus &= PMS_wakeup(PMS_handle);

	//set up the mode
	switch (mode_temp_holder) {
		case PMS_MODE_ACTIVE:
			// sensor wake up is in active mode by default - do nothing here
			break;
		case PMS_MODE_PASSIVE:
			initstatus &= PMS_swmode(PMS_handle, PMS_MODE_PASSIVE);
			break;
		default:
			break;
	}
	return initstatus;
}
/* PMS_Sleep
 * @param PMS_handle_t struct
 * @retval: status
 *
 * Send out sleep command, sleep state also can be archived with setting the SET pin to LOW
 * This function will try to sleep sensor 5 times, after 5 time with no reply, return FAIL state
 *
 * This library doesn't support Set & Reset pin of the sensor - only support control via uart
 *
 * */
int PMS_sleep(PMS_handle_t *PMS_handle)
{
	uint8_t max_try = 5;

	while(1)
	{
		uint8_t sleep_ackbuf[8] = {0};
		PMS_handle->write(cmd_sleep,sizeof(cmd_sleep),WRITE_TIMEOUT, NULL);
		PMS_handle->read(sleep_ackbuf,sizeof(sleep_ackbuf),READ_TIMEOUT, NULL);
		max_try--;
		if(_PMS_checkarray(sleep_ackbuf,sleep_ack,8) || !max_try)
		{
			break;
		}
	}
	if(max_try)
	{
		_sleepflag = 1;
		return 1;
	}
	else
	{
		return 0;
	}
}

/* PMS_wakeup
 * @param PMS_handle_t struct
 * @retval: status
 *
 * Send wake-up command to sensor, when wake-up, sensor is in mode active by default
 * This function will try to wake-up sensor 5 times, after 5 time with no reply, return FAIL state
 * (define of mode active - look up in @PMS_MODE_define)
 *
 * After wake-up, its recommended to wait at least 30 seconds to read any data
 * */
int PMS_wakeup(PMS_handle_t *PMS_handle)
{
	uint8_t max_try = 5;

	while(1)
	{
		uint8_t wakeup_ackbuf[32] = {0};
		PMS_handle->write(cmd_wakeup,sizeof(cmd_wakeup),WRITE_TIMEOUT, NULL);
		PMS_handle->read(wakeup_ackbuf,sizeof(wakeup_ackbuf),READ_TIMEOUT, NULL);
		max_try--;
		if(_PMS_checksum(wakeup_ackbuf,32) || !max_try)
		{
			break;
		}
	}

	if(max_try)
	{
		PMS_handle->mode = PMS_MODE_ACTIVE;
		_sleepflag = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}

/* PMS_swmode
 * @param PMS_handle_t struct
 * @param uint8_t mode   (look up in @mode_define)
 * @retval: status
 *
 *	This function will change the mode of sensor ACTIVE <-> PASSIVE
 *	This function will try to change mode 5 times, after 5 time with no reply, return FAIL state
 *
 * */
int PMS_swmode(PMS_handle_t *PMS_handle, uint8_t mode)
{
	if(!_sleepflag)
	{
		uint8_t max_try = 5;
		switch(mode){
		case PMS_MODE_ACTIVE:
			while(1)
			{
				uint8_t active_ackbuf[8] = {0};
				PMS_handle->write(mode_active,sizeof(mode_active),WRITE_TIMEOUT, NULL);
				PMS_handle->read(active_ackbuf,sizeof(active_ackbuf),READ_TIMEOUT, NULL);
				max_try--;
				if(_PMS_checkarray(active_ackbuf,active_ack,8) || !max_try)
				{
					break;
				}
			}
			break;
		case PMS_MODE_PASSIVE:
			while(1)
			{
				uint8_t passive_ackbuf[8] = {0};
				PMS_handle->write(mode_passive,sizeof(mode_passive),WRITE_TIMEOUT, NULL);
				PMS_handle->read(passive_ackbuf,sizeof(passive_ackbuf),READ_TIMEOUT, NULL);
				max_try--;
				if(_PMS_checkarray(passive_ackbuf,passive_ack,8) || !max_try)
				{
					break;
				}
			}
			break;
		default:
			break;
		}

		if(max_try)
		{
			PMS_handle->mode = mode;
			return 1;
		}
		else
		{
			return 0;
		}
	}
	return 1;
}
/* PMS_read
 * @param PMS_handle_t struct
 * @retval: status
 *
 *	This function read data from sensor and store them to PMS_handle
 *	User can access data by reading from PMS_handle
 *	This function will try to read data 2 times, after 2 time with no reply or fail data, return FAIL state
 *
 * */
int PMS_read(PMS_handle_t *PMS_handle)
{

	if(!_sleepflag)
	{
		uint8_t rxbuf[32] = {0};
		uint8_t max_try = 2;
		switch(PMS_handle->mode)
		{
		case PMS_MODE_PASSIVE:
			while(1)
			{
				PMS_handle->write(read_passive,sizeof(read_passive),WRITE_TIMEOUT, NULL);
				PMS_handle->read(rxbuf,sizeof(rxbuf),READ_TIMEOUT, NULL);
				max_try--;
				if( (_PMS_checksum(rxbuf,32)) || (!max_try))
				{
					break;
				}
			}
			break;
		case PMS_MODE_ACTIVE:
			while(1)
			{
				PMS_handle->read(rxbuf,sizeof(rxbuf),READ_TIMEOUT, NULL);
				max_try--;
				if( (_PMS_checksum(rxbuf,32)) || (!max_try))
				{
					break;
				}
			}
			break;
		default:
			break;
		}
		if(max_try)
		{
			PMS_handle->data.PM1_0_factory		= (rxbuf[4]	<<8) + rxbuf[5];
			PMS_handle->data.PM2_5_factory 		= (rxbuf[6]	<<8) + rxbuf[7];
			PMS_handle->data.PM10_factory 		= (rxbuf[8]	<<8) + rxbuf[9];
			PMS_handle->data.PM1_0_atmospheric 	= (rxbuf[10]<<8) + rxbuf[11];
			PMS_handle->data.PM2_5_atmospheric 	= (rxbuf[12]<<8) + rxbuf[13];
			PMS_handle->data.PM10_atmospheric	= (rxbuf[14]<<8) + rxbuf[15];
			PMS_handle->data.density_0_3um		= (rxbuf[16]<<8) + rxbuf[17];
			PMS_handle->data.density_0_5um 		= (rxbuf[18]<<8) + rxbuf[19];
			PMS_handle->data.density_1_0um 		= (rxbuf[20]<<8) + rxbuf[21];
			PMS_handle->data.density_2_5um		= (rxbuf[22]<<8) + rxbuf[23];
			PMS_handle->data.density_5_0um 		= (rxbuf[24]<<8) + rxbuf[25];
			PMS_handle->data.density_10um 		= (rxbuf[26]<<8) + rxbuf[27];
			return 1;
		}
		else
		{
			return 0;
		}
	}
	return 1;
}
/* _PMS_checksum - private function
 * @param uint8_t *rxbuf
 * @param uint8_t element
 * @retval: status
 *
 * Perform checksum for received message, message must start with 0x42
 *
 * */
int _PMS_checksum(uint8_t *rxbuf,uint8_t element)
{
	if(rxbuf[0]==0x42)
	{
		uint16_t sum = 0;
		uint16_t checksum = 0;
		for (uint8_t var = 0; var < element-3; var++)
		{
			 sum += (uint16_t)rxbuf[var];
		}
		checksum = rxbuf[element-2];
		checksum = checksum << 8UL;
		checksum |= rxbuf[element-1];

		if (sum == checksum)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}
}
/* _PMS_checkarray - private function
 * @param uint8_t *rxbuf
 * @param uint8_t *ack
 * @param uint8_t element
 * @retval: status
 *
 * Compare received message with acknowledge message and return it's a match or not
 *
 * */
int _PMS_checkarray(uint8_t *rxbuf, uint8_t *ack, uint8_t element)
{
  uint8_t pos;
  for(pos = 0; pos < element; pos++)
  {
    if (rxbuf[pos] != ack[pos])
    return 0;
  }
  return 1;
}
