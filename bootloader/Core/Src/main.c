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
#include "usart_debuger.h"
#include "string.h"
#include "bootloader.h"
#include "W25Qxx.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ALLOCATED_SIZE			512		//512 KB for each Application (2048 pages or 128 sector)
#define SECTOR_FIRMWARE1		0		//Number of Sector Allocated to the first firmware on W25Q32
#define SECTOR_FIRMWARE2		128		//Number of Sector Allocated to the Second firmware on W25Q32
#define ALLOCATED_SECTOR		128		//Numbers of sector for each firmware
#define CHUNCKSIZE 				1024	//Batch size for reading data from W25Q
#define FIRMWARE_SIZE			8428	//Size of the Firmware

#define FIRMWARE1_PAGE_W25Q		0		//First Page of W25Q Flash for First firmware
#define FIRMWARE2_PAGE_W25Q		2048	//First Page of W25Q Flash for Second firmware

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;				// SPI connected to W25Q

UART_HandleTypeDef huart3;				// Debugging USART

/* USER CODE BEGIN PV */
extern char debugStr[30];
uint8_t recData[5];
uint8_t jumpTo=0;
uint32_t pageNumber = 0;
uint8_t MCUFLASHREAD[CHUNCKSIZE];
uint32_t flashSectorNumber[10] = {FLASH_SECTOR_2, FLASH_SECTOR_3, FLASH_SECTOR_4, FLASH_SECTOR_5, FLASH_SECTOR_6,
		 FLASH_SECTOR_7, FLASH_SECTOR_8, FLASH_SECTOR_9, FLASH_SECTOR_10, FLASH_SECTOR_11};
const char ascii_art[] =
				    " _________________\r\n"
				    "|# :           : #|\r\n"
				    "|  :           :  |\r\n"
				    "|  :           :  |\r\n"
				    "|  :           :  |\r\n"
				    "|  :___________:  |\r\n"
				    "|     _________   |\r\n"
				    "|    | __      |  |\r\n"
				    "|    ||  |     |  |\r\n"
				    "\\____||__|_____|__|\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3){
		if(recData[0] == '1'){
			jumpTo = 1;
		}
		else if(recData[0] == '2'){
			jumpTo = 2;
		}
		HAL_UART_Receive_IT(&huart3, recData, 1);
	}
}

// Function to copy Firmware from W25Q to Flash of the MCU
void AppCopierFromW25Q(uint32_t AppAddress, uint32_t W25QPageAddress, int32_t FirmwareSize){


	int32_t bytesremains = FirmwareSize;
	uint32_t flashAddress = AppAddress;
	uint32_t pageAddress = W25QPageAddress;

	HAL_FLASH_Unlock();
	for(uint8_t j=0; j < 10; j++){

		FLASH_Erase_Sector(flashSectorNumber[j], FLASH_VOLTAGE_RANGE_3);
		while ((FLASH->SR & FLASH_SR_BSY) != 0);      // Wait for erase to complete

	}
	HAL_FLASH_Lock();
	while(bytesremains > 0){
		W25Q_FastRead(pageAddress, 0, CHUNCKSIZE, MCUFLASHREAD);
		HAL_FLASH_Unlock();
		for(uint32_t i=0; i < CHUNCKSIZE; i++){
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flashAddress, MCUFLASHREAD[i]);
			flashAddress++;
		}
		HAL_FLASH_Lock();
		pageAddress += 4;
		bytesremains -= CHUNCKSIZE;
	}
	HAL_UART_Transmit(&huart3, (uint8_t *)"Reading Done\n", 14, 100);
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

  //Receive the command from USART
  HAL_UART_Transmit(&huart3, (uint8_t*)ascii_art, strlen(ascii_art), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart3, (uint8_t *) "Please Enter With Firmware 1/2?", 32, 100);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
  HAL_UART_Receive_IT(&huart3, recData, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch(jumpTo){
	  case 1:
		  strcpy(debugStr, "\r\nJump to Firmware1\r\n");
		  debugTransmit();
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		  HAL_Delay(1000);
		  jumpTo = 0;
		  pageNumber = SECTOR_FIRMWARE1 * 16;
		  AppCopierFromW25Q(FIRMWARE1_BASE_ADDR, pageNumber, FIRMWARE_SIZE);
		  jumpToApp(FIRMWARE1_BASE_ADDR);									//Function to Jump to the reset handler of the firmware
		  break;
	  case 2:
		  strcpy(debugStr, "\r\nJump to Firmware2\r\n");
		  debugTransmit();
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		  HAL_Delay(1000);
		  jumpTo = 0;
		  pageNumber = SECTOR_FIRMWARE2 * 16;
		  AppCopierFromW25Q(FIRMWARE2_BASE_ADDR, pageNumber, FIRMWARE_SIZE);
		  jumpToApp(FIRMWARE2_BASE_ADDR);									//Function to Jump to the reset handler of the firmware
		  break;
	  default:
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		  HAL_Delay(1000);
	  }

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
  RCC_OscInitStruct.PLL.PLLN = 64;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
