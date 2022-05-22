
/**
 * @file       bsp_modbus_master.c
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2022-05-22
 * @author     Thuan Le
 * @brief      Board Support Package Modbus Master
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_modbus_master.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
modbus_master_t modbus;

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
void bsp_modbus_master_init(void)
{
  modbus.bsp_get_tick            = bsp_get_tick;
  modbus.bsp_uart_start_transmit = bsp_uart_start_transmit;
  modbus_complete_transmit_req   = false;
  
  modbus_master_init(&modbus);
}

uint8_t bsp_modbus_master_read_input_register(uint8_t slave_id, uint16_t read_addr, uint16_t size)
{
 return modbus_master_read_input_register(&modbus, slave_id, read_addr, size);
}

uint16_t bsp_modbus_master_get_response_buffer(uint8_t index)
{
  return modbus_master_get_response_buffer(&modbus, index);
}

/* Private function definitions ---------------------------------------- */
/* End of file -------------------------------------------------------- */
