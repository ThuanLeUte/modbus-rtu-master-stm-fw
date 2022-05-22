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
#include "bsp_modbus_master.h"
#include "bsp.h"
#include <stdio.h>
#include <string.h>
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
void test2(void);

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
  bsp_modbus_master_init();
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
void test2(void)
{
  uint32_t temp = 0;

  bsp_rs485_enable_receive(false);
  bsp_rs485_enable_transmit(true);
  if (bsp_modbus_master_read_input_register(0x01, 0x0000, 0x0008) == 0x00)
  {
    data[0] = bsp_modbus_master_get_response_buffer(0);
    data[1] = bsp_modbus_master_get_response_buffer(1);
    data[2] = bsp_modbus_master_get_response_buffer(2);
    data[3] = bsp_modbus_master_get_response_buffer(3);
    data[4] = bsp_modbus_master_get_response_buffer(4);
    data[5] = bsp_modbus_master_get_response_buffer(5);
    data[6] = bsp_modbus_master_get_response_buffer(6);
    data[7] = bsp_modbus_master_get_response_buffer(7);
  }
  else
  {
    Error++;
  }

  temp = data[1] | 0x00000000;
  temp = data[0] | (temp << 16);
  V1N = *(float *)&temp;

  temp = data[3] | 0x00000000;
  temp = data[2] | (temp << 16);
  V2N = *(float *)&temp;

  temp = data[5] | 0x00000000;
  temp = data[4] | (temp << 16);
  V3N = *(float *)&temp;

  temp = data[7] | 0x00000000;
  temp = data[6] | (temp << 16);
  VAvg = *(float *)&temp;

  sprintf(buffer, "V1N: %3.2f VAC - V2N: %3.2f VAC - V3N: %3.2f VAC - VAvg: %3.2f VAC - ResDelay: 200ms - Error: %d\r\n", V1N, V2N, V3N, VAvg, Error);
  len = strlen(buffer);
  HAL_UART_Transmit(&huart3, (uint8_t *)buffer, len, 100);

  HAL_Delay(200);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == huart1.Instance)
  {
    HAL_UART_Receive_IT(&huart1, &Uart1_RxBuffer, 1);
    if (Queue_IsFull(&modbus_master_rx_queue) == 0)
      Queue_EnQueue(&modbus_master_rx_queue, Uart1_RxBuffer);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == huart1.Instance)
  {
    if (Queue_IsEmpty(&modbus_master_tx_queue) == 1)
    {
      modbus_complete_transmit_req = true;
      bsp_rs485_enable_receive(true);
    }
    else
    {
      Uart1_TxBuffer = Queue_DeQueue(&modbus_master_tx_queue);
      HAL_UART_Transmit_IT(&huart1, &Uart1_TxBuffer, 1);
    }
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
