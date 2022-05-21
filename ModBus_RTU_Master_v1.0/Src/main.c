/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_it.h"
#include "modbus_rtu_master.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RS485_EnableDIPin											GPIO_PIN_11
#define RS485_EnableROPin											GPIO_PIN_12
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Uart1_RxBuffer = 0;
uint8_t Uart1_TxBuffer = 0;
uint32_t data[32];
char buffer[128];
int len;
float temp = 53.8;
float V1N = 0, V2N = 0, V3N = 0, VAvg = 0;
uint16_t Error = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
void ModbusMaster_BeginTransmit(void);
void ModBusMasterEnableRS485Transmit(void);
void ModBusMasterDisableRS485Transmit(void);
void ModBusMasterEnableRS485Receive(void);
void ModBusMasterDisableRS485Receive(void);
void test1(void);
void test2(void);
void putstring_float_debug(void);
void putstring_int_debug(void);

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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_UART_Receive_IT(&huart1, &Uart1_RxBuffer, 1);
	ModbusMaster_Begin();
	ModbusMaster_CompleteTransmitReQ = 0x00;
	
	
	//test1();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		data[0] = 0;
		data[1] = 0;
		data[2] = 0;
		data[3] = 0;
		data[4] = 0;
		data[5] = 0;
		data[6] = 0;
		data[7] = 0;
		test2();
		//putstring_float_debug();
		//putstring_int_debug();
		
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void putstring_float_debug(void){
	sprintf(buffer, "Float Val: %f\r\n", (float)data[0]);
	len = strlen(buffer);	
	HAL_UART_Transmit(&huart3, (uint8_t *)buffer, len, 50);
}

void putstring_int_debug(void){
	sprintf(buffer, "Int Val: %d\r\n", data[0]);
	len = strlen(buffer);	
	HAL_UART_Transmit(&huart3, (uint8_t *)buffer, len, 50);
}


void test1(void){
	ModBusMasterDisableRS485Receive();
	ModBusMasterEnableRS485Transmit();
	if(ModbusMaster_ReadHoldingRegisters(0x01, 0x0008, 0x0001) == 0x00){
		data[0] = ModbusMaster_GetResponseBuffer(0);
	}
	HAL_Delay(100);
}

void test2(void){
	
	uint32_t temp = 0;
	
	ModBusMasterDisableRS485Receive();
	ModBusMasterEnableRS485Transmit();
	if(ModbusMaster_ReadInputRegisters(0x01, 0x0000, 0x0008) == 0x00){
		data[0] = ModbusMaster_GetResponseBuffer(0);
		data[1] = ModbusMaster_GetResponseBuffer(1);
		data[2] = ModbusMaster_GetResponseBuffer(2);
		data[3] = ModbusMaster_GetResponseBuffer(3);
		data[4] = ModbusMaster_GetResponseBuffer(4);
		data[5] = ModbusMaster_GetResponseBuffer(5);
		data[6] = ModbusMaster_GetResponseBuffer(6);
		data[7] = ModbusMaster_GetResponseBuffer(7);
	}
	else {
		Error++;
	}
	
	temp = data[1] | 0x00000000;
	temp = data[0] | (temp << 16);
	V1N  = *(float*)&temp;
	
	temp = data[3] | 0x00000000;
	temp = data[2] | (temp << 16);
	V2N  = *(float*)&temp;
	
	temp = data[5] | 0x00000000;
	temp = data[4] | (temp << 16);
	V3N  = *(float*)&temp;
	
	temp = data[7] | 0x00000000;
	temp = data[6] | (temp << 16);
	VAvg = *(float*)&temp;
	
	
	sprintf(buffer, "V1N: %3.2f VAC - V2N: %3.2f VAC - V3N: %3.2f VAC - VAvg: %3.2f VAC - ResDelay: 200ms - Error: %d\r\n", V1N, V2N, V3N, VAvg, Error);
	len = strlen(buffer);	
	HAL_UART_Transmit(&huart3, (uint8_t *)buffer, len, 100);
	
	HAL_Delay(200);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	
	if(huart->Instance == huart1.Instance){
		HAL_UART_Receive_IT(&huart1, &Uart1_RxBuffer, 1);
		if(Queue_IsFull(&ModbusMaster_RX_Queue) == 0){
			Queue_EnQueue(&ModbusMaster_RX_Queue, Uart1_RxBuffer);
			
		}
		else {
			//"idle();"
		}
	}
  
} 
	

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	
	if(huart->Instance == huart1.Instance){
		
		if(Queue_IsEmpty(&ModbusMaster_TX_Queue) == 1){
			//"idle()"
			//ModBusMasterDisableRS485Transmit();
			ModbusMaster_CompleteTransmitReQ = 0x01;
			ModBusMasterEnableRS485Receive();
		}
		else {
			Uart1_TxBuffer = Queue_DeQueue(&ModbusMaster_TX_Queue);
			HAL_UART_Transmit_IT(&huart1, &Uart1_TxBuffer, 1);
		}
		
	}
	
}



void ModBusMasterEnableRS485Transmit(void){
	HAL_GPIO_WritePin(GPIOA, RS485_EnableDIPin, GPIO_PIN_SET);
}

void ModBusMasterDisableRS485Transmit(void){
	HAL_GPIO_WritePin(GPIOA, RS485_EnableDIPin, GPIO_PIN_RESET);
}

void ModBusMasterEnableRS485Receive(void){
	HAL_GPIO_WritePin(GPIOA, RS485_EnableROPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, RS485_EnableDIPin, GPIO_PIN_SET);
}

void ModBusMasterDisableRS485Receive(void){
	HAL_GPIO_WritePin(GPIOA, RS485_EnableROPin, GPIO_PIN_SET);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
