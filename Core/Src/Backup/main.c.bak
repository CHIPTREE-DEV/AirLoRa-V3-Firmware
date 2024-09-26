/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "app_lorawan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "PMS5003.h"
#include "DFRobot_MultiGasSensor.h"
#include "SHT3x.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PMS_handle_t PMS_handle;
DFRobotMGS_handle_t NO2_handle;
DFRobotMGS_handle_t O3_handle;
DFRobotMGS_handle_t CO_handle;
SHT3x_handle_t SHT3x_handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int uart1_read(uint8_t *buf, uint16_t size, uint16_t timeout, void *arg);
int uart1_write(uint8_t *buf, uint16_t size, uint16_t timeout, void *arg);
int i2c2_read(uint8_t addr, uint8_t *buf, uint16_t size, uint16_t timeout, void *arg);
int i2c2_write(uint8_t addr, uint8_t *buf, uint16_t size, uint16_t timeout, void *arg);
float AverageSamples(float* samples, int numSamples);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LoRaWAN_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(SENSOR_EN_GPIO_Port, SENSOR_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(5000);

	PMS_handle.read = uart1_read;
	PMS_handle.write = uart1_write;
	PMS_handle.mode = PMS_MODE_PASSIVE;
	memset(&PMS_handle.data, 0, sizeof(PMS_data_t));
	PMS_Init(&PMS_handle);

	NO2_handle.read = i2c2_read;
	NO2_handle.write = i2c2_write;
	NO2_handle.addr = 0x75;
	memset(&NO2_handle.data, 0, sizeof(DFRobotMGS_data_t));

	O3_handle.read = i2c2_read;
	O3_handle.write = i2c2_write;
	O3_handle.addr = 0x74;
	memset(&O3_handle.data, 0, sizeof(DFRobotMGS_data_t));

	CO_handle.read = i2c2_read;
	CO_handle.write = i2c2_write;
	CO_handle.addr = 0x76;
	memset(&CO_handle.data, 0, sizeof(DFRobotMGS_data_t));

	SHT3x_handle.read = i2c2_read;
	SHT3x_handle.write = i2c2_write;
	SHT3x_handle.addr = 0x44;
	memset(&SHT3x_handle.data, 0, sizeof(SHT3x_data_t));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_LoRaWAN_Process();

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int uart1_read(uint8_t *buf, uint16_t size, uint16_t timeout, void *arg)
{
	UNUSED(arg);
	return HAL_UART_Receive(&huart1, buf, size, timeout) == HAL_OK;
}

int uart1_write(uint8_t *buf, uint16_t size, uint16_t timeout, void *arg)
{
	UNUSED(arg);
	return HAL_UART_Transmit(&huart1, buf, size, timeout) == HAL_OK;
}

int i2c2_read(uint8_t addr, uint8_t *buf, uint16_t size, uint16_t timeout, void *arg)
{
	UNUSED(arg);
	return HAL_I2C_Master_Receive(&hi2c2, addr << 1u, buf, size, timeout) == HAL_OK;
}

int i2c2_write(uint8_t addr, uint8_t *buf, uint16_t size, uint16_t timeout, void *arg)
{
	UNUSED(arg);
	return HAL_I2C_Master_Transmit(&hi2c2, addr << 1u, buf, size, timeout) == HAL_OK;
}

float AverageSamples(float* samples, int numSamples)
{
	float sum = 0;
    for (int i = 0; i < numSamples; i++)
    {
        sum += samples[i];
    }
    return (float)(sum / numSamples);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
