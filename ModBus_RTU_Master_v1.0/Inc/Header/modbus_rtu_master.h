#ifndef __MODBUS_RTU_MASTER_H
#define __MODBUS_RTU_MASTER_H

#include <stdint.h>
#include "mcu_application.h"
#include "crc_16.h"
#include "word.h"
#include "queue_circle_array.h"



extern	uint8_t 	u8ModbusADU[128];
extern  uint8_t 	u8ModbusADUSize;
extern  uint8_t 	i;
extern	uint8_t   u8Quantity;
extern  uint16_t 	u16CRC;
extern  uint8_t 	u8BytesLeft;
extern	uint32_t 	u32ModBusLoopTimeout;
extern  uint8_t 	u8ModBusStatus;
	
	
extern volatile uint8_t 	ModbusMaster_CompleteTransmitReQ;


extern struct_queue_array ModbusMaster_TX_Queue;
extern struct_queue_array ModbusMaster_RX_Queue;




void ModbusMaster_Begin(void);
void ModbusMaster_ClearADU(void);
void ModbusMaster_BeginTransmit(void);
uint16_t ModbusMaster_GetResponseBuffer(uint8_t Index);
void ModbusMaster_ClearResponseBuffer(void);
uint8_t ModbusMaster_SetTransmitBuffer(uint8_t Index, uint16_t Value);
uint32_t ModbusMaster_GetMillis(void);
uint8_t ModbusMaster_Execute_Transaction(uint8_t ModbusFunction);
uint8_t ModbusMaster_Check_SlaveResponse(uint8_t u8ModbusFunction);

uint8_t ModbusMaster_readCoils(uint8_t u8SlaveID, uint16_t u16ReadAddress, uint16_t u16BitQuantity);													//0x01
uint8_t ModbusMaster_readDiscreteInputs(uint8_t u8SlaveID, uint16_t u16ReadAddress, uint16_t u16BitByteQuantity);							//0x02
uint8_t ModbusMaster_ReadHoldingRegisters(uint8_t u8SlaveID, uint16_t u16ReadAddress, uint16_t u16BitQuantity);								//0x03
uint8_t ModbusMaster_ReadInputRegisters(uint8_t u8SlaveID, uint16_t u16ReadAddress, uint16_t u16ReadQuantity);								//0x04
uint8_t ModbusMaster_WriteSingleCoil(uint8_t u8SlaveID, uint16_t u16WriteAddress, uint8_t u8State);														//0x05
uint8_t ModbusMaster_WriteSingleRegister(uint8_t u8SlaveID, uint16_t u16WriteAddress, uint16_t u16WriteValue);								//0x06
uint8_t ModbusMaster_WriteMultipleCoils(uint8_t u8SlaveID, uint16_t u16WriteAddress, uint16_t u16BitByteQuantity); 						//0x0F
uint8_t ModbusMaster_WriteMultipleRegisters(uint8_t u8SlaveID, uint16_t u16WriteAddress, uint16_t u16WriteQuantity);  				//0x10
uint8_t ModbusMaster_MaskWriteRegister(uint8_t u8SlaveID,uint16_t u16WriteAddress, uint16_t u16AndMask, uint16_t u16OrMask); 	//0x16
uint8_t ModbusMaster_ReadWriteMultipleRegisters(uint8_t u8SlaveID, uint16_t u16ReadAddress, uint16_t u16ReadQuantity, 
																								uint16_t u16WriteAddress, uint16_t u16WriteQuantity);													//0x17						



#endif  // __MODBUS_RTU_MASTER_H

