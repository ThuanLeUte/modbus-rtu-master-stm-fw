#ifndef __MODBUS_RTU_MASTER_H
#define __MODBUS_RTU_MASTER_H

#include "mcu_application.h"
#include "util.h"
#include "queue_circle_array.h"

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

void modbus_master_init(void);
void modbus_master_clear_adu(void);
void modbus_master_start_transmit(void);
uint16_t modbus_master_get_response_buffer(uint8_t index);
uint32_t ModbusMaster_GetMillis(void);
uint8_t modbus_master_execute_transaction(uint8_t modbus_function);

uint8_t modbus_master_read_coils(uint8_t slave_id, uint16_t read_addr, uint16_t size);                                 // 0x01
uint8_t modbus_master_read_discrete_input(uint8_t slave_id, uint16_t read_addr, uint16_t size);                        // 0x02
uint8_t modbus_master_read_holding_register(uint8_t slave_id, uint16_t read_addr, uint16_t size);                      // 0x03
uint8_t modbus_master_read_input_register(uint8_t slave_id, uint16_t read_addr, uint16_t size);                        // 0x04
uint8_t modbus_master_write_single_coil(uint8_t slave_id, uint16_t write_addr, uint8_t state);                         // 0x05
uint8_t modbus_master_write_single_register(uint8_t slave_id, uint16_t write_addr, uint16_t value);                    // 0x06
uint8_t modbus_master_write_mutiple_coils(uint8_t slave_id, uint16_t write_addr, uint16_t size);                       // 0x0F
uint8_t modbus_master_write_mutiple_register(uint8_t slave_id, uint16_t write_addr, uint16_t size);                    // 0x10
uint8_t modbus_master_mask_write_register(uint8_t slave_id, uint16_t write_addr, uint16_t and_mask, uint16_t or_mask); // 0x16
uint8_t modbus_maste_read_mutiple_registers(uint8_t slave_id, uint16_t read_addr, uint16_t read_size,
                                            uint16_t write_addr, uint16_t write_size); // 0x17

#endif // __MODBUS_RTU_MASTER_H
