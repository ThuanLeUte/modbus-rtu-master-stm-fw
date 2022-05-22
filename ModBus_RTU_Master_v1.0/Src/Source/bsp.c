
/**
 * @file       bsp.c
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2021-01-23
 * @author     Thuan Le
 * @brief      Board Support Package (BSP)
 * 
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "bsp.h"

/* Private defines ---------------------------------------------------- */

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
extern UART_HandleTypeDef huart1;

/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
uint32_t bsp_get_tick(void)
{
  return HAL_GetTick();
}

void bsp_uart_start_transmit(void)
{
  __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE); // Disable transmit complete interrupts
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);   // Enable transmit complete interrupts
}

/* Private function definitions ---------------------------------------- */
/* End of file -------------------------------------------------------- */
