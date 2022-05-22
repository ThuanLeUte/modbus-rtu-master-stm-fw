/**
 * @file       modbus_master.h
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2021-04-09
 * @author     Thuan Le
 * @brief      Driver support modbus master
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __MODBUS_MASTER_H
#define __MODBUS_MASTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include "util.h"
#include "bsp.h"
#include "queue_circle_array.h"

/* Private macros ----------------------------------------------------- */
/* Public defines ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
extern uint8_t modbus_adu[128];
extern uint8_t modbus_adu_size;
extern uint8_t i;
extern uint8_t quantity;
extern uint16_t crc16;
extern uint8_t byte_left;
extern uint32_t modbus_loop_timeout;
extern uint8_t modbus_status;

extern volatile uint8_t modbus_complete_transmit_req;
extern struct_queue_array modbus_master_tx_queue;
extern struct_queue_array modbus_master_rx_queue;

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Modbus master struct
 */
typedef struct
{
  uint32_t (*bsp_get_tick) (void);
  void     (*bsp_uart_start_transmit) (void);
}
modbus_master_t;

/* Public function prototypes ----------------------------------------- */
void modbus_master_init(modbus_master_t *me);
void modbus_master_clear_adu(modbus_master_t *me);
uint16_t modbus_master_get_response_buffer(modbus_master_t *me, uint8_t index);
uint8_t modbus_master_execute_transaction(modbus_master_t *me, uint8_t modbus_function);

uint8_t modbus_master_read_coils(modbus_master_t *me, uint8_t slave_id, uint16_t read_addr, uint16_t size);
uint8_t modbus_master_read_discrete_input(modbus_master_t *me, uint8_t slave_id, uint16_t read_addr, uint16_t size);
uint8_t modbus_master_read_holding_register(modbus_master_t *me, uint8_t slave_id, uint16_t read_addr, uint16_t size);
uint8_t modbus_master_read_input_register(modbus_master_t *me, uint8_t slave_id, uint16_t read_addr, uint16_t size);
uint8_t modbus_master_write_single_coil(modbus_master_t *me, uint8_t slave_id, uint16_t write_addr, uint8_t state);
uint8_t modbus_master_write_single_register(modbus_master_t *me, uint8_t slave_id, uint16_t write_addr, uint16_t value);
uint8_t modbus_master_write_mutiple_coils(modbus_master_t *me, uint8_t slave_id, uint16_t write_addr, uint16_t size);
uint8_t modbus_master_write_mutiple_register(modbus_master_t *me, uint8_t slave_id, uint16_t write_addr, uint16_t size);
uint8_t modbus_master_mask_write_register(modbus_master_t *me, uint8_t slave_id, uint16_t write_addr, uint16_t and_mask, uint16_t or_mask);
uint8_t modbus_maste_read_mutiple_registers(modbus_master_t *me, uint8_t slave_id, uint16_t read_addr, uint16_t read_size,
                                            uint16_t write_addr, uint16_t write_size);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __MODBUS_MASTER_H

/* End of file -------------------------------------------------------- */
