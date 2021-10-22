/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "co2_sensor.h"
#include <string.h>
#include <stdio.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


uint8_t cnt_measurement[8];
uint8_t mess_data[18];
uint8_t receive[1];


unsigned char ready;
float temperature;
float co2concentration;
unsigned int co2_value;
unsigned int temperature_temp;
unsigned int humidity_temp ;
float humidity ;






/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */



/*** Check the if I2C communication is set by toggling the  blue LED ==> the sensor is given the right address ***/


  if (HAL_I2C_IsDeviceReady(&hi2c1,senor_address,2,HAL_MAX_DELAY) == HAL_OK)

 	   		{

 	   	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
 	   		}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {









co2_sensor_measure_start();
co2_sensor_get_ready_status();
co2_sensor_measure_read();




























    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



/**** Start measurement ****/


void co2_sensor_measure_start(void)

{

		   	  cnt_measurement[0]=0x00;
		   	  cnt_measurement[1]=0x10;
		   	  cnt_measurement[2]=0x00;
		   	  cnt_measurement[3]=0x00;
		   	  cnt_measurement[4]=0x81;

		   	  HAL_I2C_Master_Transmit(&hi2c1,sensor_address,cnt_measurement,5,1000);



}


/***************************** Get ready Status***********************/
/***** It is essential to know when we can have data to read **********/

void co2_sensor_get_ready_status(void)

{

	   	cnt_measurement[0]=0x02;
	    cnt_measurement[1]=0x02;


	   	  if (HAL_I2C_Master_Transmit(&hi2c1,sensor_address,cnt_measurement,2,HAL_MAX_DELAY)== HAL_OK)

	   	  {
	   	    HAL_I2C_Master_Receive(&hi2c1
	   	    		,sensor_address_read,receive,3,100);

	   	    ready = receive[1];
	   	  }
}


/*************** Read and display measured data ( CO2 values in PPM ) ******************/

void co2_sensor_measure_read(void)
{


	       cnt_measurement[0]=0x03;
	   	   cnt_measurement[1]=0x00;

	   	  if (ready == 0x01 ){     //always check if measurement is ready


	   	    HAL_I2C_Master_Transmit(&hi2c1,sensor_address,cnt_measurement,2,1000);
	   	    HAL_I2C_Master_Receive(&hi2c1,sensor_address_read,mess_data,18,100);

	   	    // cast 4 bytes to one unsigned 32 bit integer
	   	    co2_value = (unsigned int)((((unsigned int)mess_data[0]) << 24)|
	   	                              (((unsigned int)mess_data[1]) << 16) |
	   	                                (((unsigned int)mess_data[3]) << 8) |
	   	                                  ((unsigned int)mess_data[4]));

	   	    // cast unsigned 32 bit integer to 32 bit float

	   	 co2concentration= *(float*)&co2_value;


		  temperature_temp = (unsigned int)((((unsigned int)mess_data[6]) << 24)|
		 	                                      (((unsigned int)mess_data[7]) << 16) |
		 	                                        (((unsigned int)mess_data[9]) << 8) |
		 	                                          ((unsigned int)mess_data[10]));
		    temperature = *(float*)&temperature_temp;


	   	  humidity_temp = (unsigned int)((((unsigned int)mess_data[12]) << 24)|
	 	                                   (((unsigned int)mess_data[13]) << 16) |
	 	                                     (((unsigned int)mess_data[15]) << 8) |
	 	                                       ((unsigned int)mess_data[16]));
	 	    humidity = *(float*)&humidity_temp;



	   	 // convert the content of co2concentration to string and put it in the buffer mess_data

	   	 sprintf((char*)mess_data,"%u PPM\r\n",
	   		    	  	              ((unsigned int)co2concentration));



	   	 // Display the current Co2 measurement in PPM ( Part per Million)


	 	HAL_UART_Transmit(&huart2,mess_data,strlen((char*)mess_data), HAL_MAX_DELAY);




	 	 sprintf((char*)mess_data,"%u C\r\n",
		   		    	  	              ((unsigned int)temperature));






		   	 // Display the current Co2 measurement in PPM ( Part per Million)


		 	HAL_UART_Transmit(&huart2,mess_data,strlen((char*)mess_data), HAL_MAX_DELAY);




 sprintf((char*)mess_data,"%u H\r\n",
		   		    	  	              ((unsigned int)humidity));


HAL_UART_Transmit(&huart2,mess_data,strlen((char*)mess_data), HAL_MAX_DELAY);






















 }

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
