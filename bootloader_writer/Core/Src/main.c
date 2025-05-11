/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "W25Qxx.h"		//Library for working with W25Q
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHUNKSIZE 				1024
#define FIRMWARE1_BASE_ADDR 	0x08008000
#define FIRMWARE2_BASE_ADDR 	0x08040000

#define ALLOCATED_SIZE			512	//512 KB for each Application (2048 pages or 128 sector)
#define SECTOR_FIRMWARE1		0
#define SECTOR_FIRMWARE2		128
#define ALLOCATED_SECTOR		20

#define FIRMWARE1_SIZE			8428
#define FIRMWARE2_SIZE			8428

#define FIRMWARE1_PAGE_W25Q		0
#define FIRMWARE2_PAGE_W25Q		2048



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart3;


/* USER CODE BEGIN PV */
uint8_t MCUFLASHREAD[CHUNKSIZE];
uint8_t USART_CMD_REC[2];
uint8_t	CommandNumber = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/*
 * ***********************************************************
 * Function to Read program data from flash memory of STM32
 * and write it into the W25Q flash memory
 * ***********************************************************
 * */
void WRITE_TO_FLASHW25Q(uint32_t AppAddress, uint32_t W25QPageAddress, int32_t FirmwareSize){

	int32_t bytesremains = FirmwareSize;
	uint32_t flashAddress = AppAddress;
	uint32_t pageAddress = W25QPageAddress;
	uint32_t SectorOffset = 0;
	// Sector offset for storing firmwares
	if(AppAddress == FIRMWARE1_BASE_ADDR){
		SectorOffset = SECTOR_FIRMWARE1;
	}
	else if(AppAddress == FIRMWARE2_BASE_ADDR){
		SectorOffset = SECTOR_FIRMWARE2;
	}

	// Erasing allocated sectors to the firmware
	for(uint32_t j=0; j < ALLOCATED_SECTOR; j++){
		W25Q_Erase_Sector(SectorOffset +j);
	}

	//Writing Flash program on W25Q
	while(bytesremains > 0){
		HAL_FLASH_Unlock();
		for(uint32_t i=0; i < CHUNKSIZE; i++){
			MCUFLASHREAD[i] = *(__IO uint8_t*)flashAddress; // Reading each address of flash and store it in an array
			flashAddress++;
		}
		HAL_FLASH_Lock();
		W25Q_Write_Page(pageAddress, 0, CHUNKSIZE, MCUFLASHREAD); //writing data on the flash
		pageAddress += 4;			//increasing the pagenumber by 4 (each chunk is 1028 that is 4*256 (each page is 256 bytes)
		bytesremains -= CHUNKSIZE;	//subtract chunk from remained data to transfer

	}

	HAL_UART_Transmit(&huart3, (uint8_t *)"Writing Done\n", 14, 100);


}


/*
 * ***********************************************************
 * Function to Read program data from W25Q
 * and write it into the flash memory of STM32
 * ***********************************************************
 * */
void WRITE_FROM_W25_TO_MCU(uint32_t AppAddress, uint32_t W25QPageAddress, int32_t FirmwareSize){


	int32_t bytesremains = FirmwareSize;
	uint32_t flashAddress = AppAddress;
	uint32_t pageAddress = W25QPageAddress;
	while(bytesremains > 0){
		W25Q_FastRead(pageAddress, 0, CHUNKSIZE, MCUFLASHREAD);
		HAL_FLASH_Unlock();
		for(uint32_t i=0; i < CHUNKSIZE; i++){
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flashAddress, MCUFLASHREAD[i]);
			flashAddress++;
		}
		HAL_FLASH_Lock();
		pageAddress += 4;
		bytesremains -= CHUNKSIZE;
	}
	HAL_UART_Transmit(&huart3, (uint8_t *)"Reading Done\n", 14, 100);
}



/*
 * ***********************************************************
 * USART Function for reading command for READ/WRITE
 * W1: write firmware1 to W25Q
 * W2: write firmware2 to W25Q
 * R1: Read firmware1 from W25Q and program Flash of the MCU
 * R2: Read firmware2 from W25Q and program Flash of the MCU
 * ***********************************************************
 * */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3){
		if(USART_CMD_REC[0] == 'R'){
			switch(USART_CMD_REC[1]){
			case '1':
				WRITE_FROM_W25_TO_MCU(FIRMWARE1_BASE_ADDR, FIRMWARE1_PAGE_W25Q, FIRMWARE1_SIZE);
				break;
			case '2':
				WRITE_FROM_W25_TO_MCU(FIRMWARE2_BASE_ADDR, FIRMWARE2_PAGE_W25Q, FIRMWARE2_SIZE);
				break;
			default:
				HAL_UART_Transmit(&huart3, (uint8_t *)"Please Send correct command\n", 29, 100);
			}

		}
		else if(USART_CMD_REC[0] == 'W'){
			switch(USART_CMD_REC[1]){
			case '1':
				WRITE_TO_FLASHW25Q(FIRMWARE1_BASE_ADDR, FIRMWARE1_PAGE_W25Q, FIRMWARE1_SIZE);
				break;
			case '2':
				WRITE_TO_FLASHW25Q(FIRMWARE2_BASE_ADDR, FIRMWARE2_PAGE_W25Q, FIRMWARE2_SIZE);
				break;
			default:
				HAL_UART_Transmit(&huart3, (uint8_t *)"Please Send correct command\n", 29, 100);
			}

		}
		HAL_UART_Receive_IT(&huart3, USART_CMD_REC, 2);
	}
}

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
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  W25Q_Reset();
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
  HAL_UART_Transmit(&huart3, (uint8_t *)"Choose a command (R/W)(1/2)?", 29, 100);
  HAL_UART_Receive_IT(&huart3, USART_CMD_REC, 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
