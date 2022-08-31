/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CDD_L9963_drv_interface.h"
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

SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t t = 0;
  L9963_Test_Init();
	//uint8_t return_code = true;
	//uint8 res = true;
	
	//L9963_SPI_Rx_Inst_t  *spi_rx;
	//spi_rx = 0;
	//uint8_t op_code[1] = {0xff};
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	
	uint8_t spi_mosi[5];

	
  while (1)
  {
		//uint8_t res = true;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
		//GCDD_L9963_CellVol_Burst_Read();
		//L9963_SPI_Rx_Inst_t spi_rx_t;
		
	
		//L9963_SPI_CS_SELECT;
		
		//HAL_SPI_Transmit(&hspi2,&t,1,100);
		
		
		//L9963_SPI_CS_UNSELECT;
		
//		//	L9963_SPI_CS_SELECT;
//			
//		//res = 	HAL_SPI_TransmitReceive( &hspi2, &spi_mosi[0], (((uint8 *) spi_rx) ), 5,  100);
//			
//	//	GCDD_L9963_TxData(&spi_mosi[0], (((uint8 *) spi_rx) ), 5);
//		
//		L9963_SPI_CS_UNSELECT;
//	HAL_Delay(319); /*Total Delay time need to be 219us */
//	L9963_SPI_CS_SELECT;
//		//	HAL_SPI_TransmitReceive( &hspi2, &spi_mosi[0], (((uint8 *) spi_rx) ), 5,  100);
//		HAL_SPI_Transmit(&hspi2, spi_mosi,5,100);
//		
//		L9963_SPI_CS_UNSELECT;
//	HAL_Delay(319);
//	spi_mosi[0] = 0xC0; /*first 24 bit is reseved*/
//	spi_mosi[1] = 0x04;
//	spi_mosi[2] = 0x0F;
//	spi_mosi[3] = 0x00;
//	spi_mosi[4] = 0x28;
//	
//		L9963_SPI_CS_SELECT;
//		//	HAL_SPI_TransmitReceive( &hspi2, &spi_mosi[0], (((uint8 *) spi_rx) ), 5,  100);
//			HAL_SPI_Transmit(&hspi2, spi_mosi,5,100);
//	L9963_SPI_CS_UNSELECT;
//	HAL_Delay(2519); /*Total Delay time need to be 219us */
//	
   // Delay(6517);
//		//uint8 return_code = true;'
//	//	return_code &= L9963_Single_Read(GEN_STATUS_ADD, 0xD, &spi_rx_t);

//		
//		
		//HAL_Delay(50);
		
		
		
	/////////////////////Ä£Äâ////////////////////////////////	
	//	uint8_t spi_mosi[5];
//	spi_mosi[0] = 0x82; /*first 24 bit is reseved*/
//	  spi_mosi[1] = 0x04;
//	  spi_mosi[2] = 0x00;
//	  spi_mosi[3] = 0x00;
//	  spi_mosi[4] = 0x17;
//		
//		Delay(2000);
//		
//		L9963_SPI_CS_SELECT;
//			HAL_SPI_Transmit(&hspi2,&spi_mosi[0],5,100);
//L9963_SPI_CS_UNSELECT;
//	
//	Delay(819);
//	
//		L9963_SPI_CS_SELECT;
//			HAL_SPI_Transmit(&hspi2,&spi_mosi[0],5,100);
//L9963_SPI_CS_UNSELECT;
//	Delay(819);
//	
//	
//	
//	
//	Delay(4590);
//	
//	
//	
//		L9963_SPI_CS_SELECT;
//			HAL_SPI_Transmit(&hspi2,&spi_mosi[0],5,100);
//L9963_SPI_CS_UNSELECT;
//	
//	Delay(319);
//	
//	L9963_SPI_CS_SELECT;
//			HAL_SPI_Transmit(&hspi2,&spi_mosi[0],5,100);
//L9963_SPI_CS_UNSELECT;
//	Delay(319);
//	
//	
//	Delay(6590);
//	
//	  spi_mosi[0] = 0xC0; /*first 24 bit is reseved*/
//	  spi_mosi[1] = 0x04;
//	  spi_mosi[2] = 0x0F;
//	  spi_mosi[3] = 0x00;
//	  spi_mosi[4] = 0x28;
//	
//	
//	    L9963_SPI_CS_SELECT;
//			HAL_SPI_Transmit(&hspi2,&spi_mosi[0],5,100);
//      L9963_SPI_CS_UNSELECT;
//	
//	//Delay(319);
//	Delay(2519);
//	  spi_mosi[0] = 0xC2; /*first 24 bit is reseved*/
//	  spi_mosi[1] = 0x0C;
//	  spi_mosi[2] = 0x00;
//	  spi_mosi[3] = 0x00;
//	  spi_mosi[4] = 0x0C;
//	
//	L9963_SPI_CS_SELECT;
//			HAL_SPI_Transmit(&hspi2,&spi_mosi[0],5,100);
//L9963_SPI_CS_UNSELECT;
//	
//	//Delay(319);
//	Delay(2519);
//	
//	
//	  spi_mosi[0] = 0x82; /*first 24 bit is reseved*/
//	  spi_mosi[1] = 0x04;
//	  spi_mosi[2] = 0x00;
//	  spi_mosi[3] = 0x00;
//	  spi_mosi[4] = 0x17;
//		
//	
//	
//	L9963_SPI_CS_SELECT;
//			HAL_SPI_Transmit(&hspi2,&spi_mosi[0],5,100);
//L9963_SPI_CS_UNSELECT;
//	
//	Delay(319);
//	Delay(2519);
//	
//	L9963_SPI_CS_SELECT;
//			HAL_SPI_Transmit(&hspi2,&spi_mosi[0],5,100);
//L9963_SPI_CS_UNSELECT;
//	
//	Delay(319);
//		Delay(2519); 
//		
//		
//	
//			Delay(65900);
//			
			/////////////////////Ä£Äâ////////////////////////////////	
		
		
		
		
		
		
		
	GCDD_L9963_CellVol_Burst_Read();

		//HAL_SPI_Transmit(&hspi2,&t,1,100);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 4096;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

	
//	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
//	hspi2.Init.CRCLength = SPI_CRC_LENGTH_6BIT;
//  hspi2.Init.CRCPolynomial = 0x59;
//	
  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
