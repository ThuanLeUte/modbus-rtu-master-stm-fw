#include "modbus_rtu_master.h"

extern UART_HandleTypeDef huart1;

#define ModBusExCepIllegalFunction      0x01
#define ModBusExCepIllegalDataAddress   0x02
#define ModBusExCepIllegalDataValue     0x03
#define ModBusExCepSlaveDeviceFailure   0x04

#define ModBusStatusTransactionSuccess  0x00
#define ModBusStatusInvalidSlaveID      0xE0
#define ModBusStatusInvalidFunction     0xE1
#define ModBusStatusExpiredTimedOut     0xE2
#define ModBusStatusInvalidCRC          0xE3

// Modbus function codes for bit access
#define MODBUS_FUNCTION_READ_COILS                    (0x01) // Modbus function 0x01 Read Coils
#define MODBUS_FUNCTION_READ_DISCRETE_INPUTS          (0x02) // Modbus function 0x02 Read Discrete Inputs
#define MODBUS_FUNCTION_WRITE_SINGLE_COIL             (0x05) // Modbus function 0x05 Write Single Coil
#define MODBUS_FUNCTION_WRITE_MUTIPLE_COILS           (0x0F) // Modbus function 0x0F Write Multiple Coils

// Modbus function codes for 16 bit access
#define MODBUS_FUNCTION_READ_HOLDING_REGISTERS        (0x03) // Modbus function 0x03 Read Holding Registers
#define MODBUS_FUNCTION_READ_INPUT_REGISTERS          (0x04) // Modbus function 0x04 Read Input Registers
#define MODBUS_FUNCTION_WRITE_SINGLE_REGISTER         (0x06) // Modbus function 0x06 Write Single Register
#define MODBUS_FUNCTION_WRITE_MUTIPLE_REGISTERS       (0x10) // Modbus function 0x10 Write Multiple Registers
#define MODBUS_FUNCTION_MASK_WRITE_REGISTERS          (0x16) // Modbus function 0x16 Mask Write Register
#define MODBUS_FUNCTION_READ_WRITE_MUTIPLE_REGISTERS  (0x17) // Modbus function 0x17 Read Write Multiple Registers

#define MODBUS_RESPONSE_TIMEOUT                       (2000)
#define MODBUS_COMPLETE_TRANSACTION                   (0x01)
#define MODBUS_NOT_COMPLETE_TRANSACTION               (0x00)

struct_queue_array modbus_master_tx_queue;
struct_queue_array modbus_master_rx_queue;

uint8_t u8ModbusMaster_QueueTXBuffer[64];
uint8_t u8ModbusMaster_QueueRXBuffer[64];

static const uint8_t u8Modbus_TransmitBufferSize = 64;
static const uint8_t u8Modbus_ResponseBufferSize = 64;

uint16_t u16Modbus_TransmitBuffer[u8Modbus_TransmitBufferSize]; // data want to write to slave
uint16_t u16Modbus_ResponseBuffer[u8Modbus_ResponseBufferSize]; // data response from slave
uint8_t u8Modbus_TransmitBufferLengh = 0;
uint8_t u8Modbus_ResponseBufferLengh = 0;

volatile uint8_t ModbusMaster_CompleteTransmitReQ = 0;
uint8_t u8ModBusSlaveID;
uint16_t u16ModbusReadAddress;
uint16_t u16ModbusReadQuantity;
uint16_t u16ModbusWriteAddress;
uint16_t u16ModbusWriteQuantity;

// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
uint8_t u8ModbusADU[128];
uint8_t u8ModbusADUSize = 0;
uint8_t i = 0;
uint8_t u8Quantity = 0;
uint16_t u16CRC = 0xffff;
uint8_t u8BytesLeft = 0;
uint32_t u32ModBusLoopTimeout = 0;
uint8_t u8ModBusStatus = ModBusStatusTransactionSuccess;
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

void ModbusMaster_Begin(void)
{
  u8Modbus_TransmitBufferLengh = 0;
  u8Modbus_ResponseBufferLengh = 0;
  Queue_Initialize(&modbus_master_tx_queue, u8ModbusMaster_QueueTXBuffer, sizeof(u8ModbusMaster_QueueTXBuffer));
  Queue_Initialize(&modbus_master_rx_queue, u8ModbusMaster_QueueRXBuffer, sizeof(u8ModbusMaster_QueueRXBuffer));
}

void ModbusMaster_ClearADU(void)
{
  for (uint8_t k = 0; k < 128; k++)
  {
    u8ModbusADU[k] = 0;
  }
}

uint32_t ModbusMaster_GetMillis(void)
{
  return HAL_GetTick();
}

void ModbusMaster_BeginTransmit(void)
{
  __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE); // disable transmit complete interrupts
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);   // enable transmit complete interrupts
}

uint16_t ModbusMaster_GetResponseBuffer(uint8_t Index)
{
  if (Index < u8Modbus_ResponseBufferSize)
  {
    return u16Modbus_ResponseBuffer[Index];
  }
}

void ModbusMaster_ClearResponseBuffer(void)
{
  uint8_t i;

  for (i = 0; i < u8Modbus_ResponseBufferSize; i++)
  {
    u16Modbus_ResponseBuffer[i] = 0;
  }
}

uint8_t ModbusMaster_SetTransmitBuffer(uint8_t Index, uint16_t Value)
{
  if (Index < u8Modbus_TransmitBufferSize)
  {
    u16Modbus_TransmitBuffer[Index] = Value;
    return ModBusStatusTransactionSuccess;
  }
  else
  {
    return ModBusExCepIllegalDataAddress;
  }
}

void ModbusMaster_ClearTransmitBuffer(void)
{
  uint8_t i;

  for (i = 0; i < u8Modbus_TransmitBufferSize; i++)
  {
    u16Modbus_TransmitBuffer[i] = 0;
  }
}

uint8_t ModbusMaster_Execute_Transaction(uint8_t u8ModbusFunction)
{

  u8ModbusADUSize = 0;
  i = 0;
  u8Quantity = 0;
  u16CRC = 0xffff;
  u8BytesLeft = 0;
  u32ModBusLoopTimeout = 0;
  u8ModBusStatus = ModBusStatusTransactionSuccess;

  // xxxxxxxxxxxxxxxxxxxxxxx setup ADU frame xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //  assemble Modbus Request Application Data Unit
  u8ModbusADU[u8ModbusADUSize++] = u8ModBusSlaveID;
  u8ModbusADU[u8ModbusADUSize++] = u8ModbusFunction;

  switch (u8ModbusFunction)
  {
  case MODBUS_FUNCTION_READ_COILS:
  case MODBUS_FUNCTION_READ_DISCRETE_INPUTS:
  case MODBUS_FUNCTION_READ_HOLDING_REGISTERS:
  case MODBUS_FUNCTION_READ_INPUT_REGISTERS:
  case MODBUS_FUNCTION_READ_WRITE_MUTIPLE_REGISTERS:

    u8ModbusADU[u8ModbusADUSize++] = highByte(u16ModbusReadAddress);
    u8ModbusADU[u8ModbusADUSize++] = lowByte(u16ModbusReadAddress);
    u8ModbusADU[u8ModbusADUSize++] = highByte(u16ModbusReadQuantity);
    u8ModbusADU[u8ModbusADUSize++] = lowByte(u16ModbusReadQuantity);

    break;
  }

  switch (u8ModbusFunction)
  {
  case MODBUS_FUNCTION_WRITE_SINGLE_COIL:
  case MODBUS_FUNCTION_MASK_WRITE_REGISTERS:
  case MODBUS_FUNCTION_WRITE_MUTIPLE_COILS:
  case MODBUS_FUNCTION_WRITE_SINGLE_REGISTER:
  case MODBUS_FUNCTION_WRITE_MUTIPLE_REGISTERS:
  case MODBUS_FUNCTION_READ_WRITE_MUTIPLE_REGISTERS:

    u8ModbusADU[u8ModbusADUSize++] = highByte(u16ModbusWriteAddress);
    u8ModbusADU[u8ModbusADUSize++] = lowByte(u16ModbusWriteAddress);

    break;
  }

  switch (u8ModbusFunction)
  {
  case MODBUS_FUNCTION_WRITE_SINGLE_COIL:
    u8ModbusADU[u8ModbusADUSize++] = highByte(u16ModbusWriteQuantity);
    u8ModbusADU[u8ModbusADUSize++] = lowByte(u16ModbusWriteQuantity);
    break;

  case MODBUS_FUNCTION_WRITE_SINGLE_REGISTER:
    u8ModbusADU[u8ModbusADUSize++] = highByte(u16Modbus_TransmitBuffer[0]);
    u8ModbusADU[u8ModbusADUSize++] = lowByte(u16Modbus_TransmitBuffer[0]);
    break;

  case MODBUS_FUNCTION_WRITE_MUTIPLE_COILS:
    u8ModbusADU[u8ModbusADUSize++] = highByte(u16ModbusWriteQuantity);
    u8ModbusADU[u8ModbusADUSize++] = lowByte(u16ModbusWriteQuantity);

    if (u16ModbusWriteQuantity % 8)
    {
      u8Quantity = (u16ModbusWriteQuantity >> 3) + 1;
    }
    else
    {
      u8Quantity = u16ModbusWriteQuantity >> 3;
    }

    for (i = 0; i < u8Quantity; i++)
    {

      switch (i % 2)
      {
      case 0:
        u8ModbusADU[u8ModbusADUSize++] = lowByte(u16Modbus_TransmitBuffer[i >> 1]);
        break;

      case 1:
        u8ModbusADU[u8ModbusADUSize++] = highByte(u16Modbus_TransmitBuffer[i >> 1]);
        break;
      }
    }

    break;

  case MODBUS_FUNCTION_WRITE_MUTIPLE_REGISTERS:
  case MODBUS_FUNCTION_READ_WRITE_MUTIPLE_REGISTERS:
    for (i = 0; i < lowByte(u16ModbusWriteQuantity); i++)
    {
      u8ModbusADU[u8ModbusADUSize++] = highByte(u16Modbus_TransmitBuffer[i]);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(u16Modbus_TransmitBuffer[i]);
    }
    break;

  case MODBUS_FUNCTION_MASK_WRITE_REGISTERS:
    u8ModbusADU[u8ModbusADUSize++] = highByte(u16Modbus_TransmitBuffer[0]);
    u8ModbusADU[u8ModbusADUSize++] = lowByte(u16Modbus_TransmitBuffer[0]);
    u8ModbusADU[u8ModbusADUSize++] = highByte(u16Modbus_TransmitBuffer[1]);
    u8ModbusADU[u8ModbusADUSize++] = lowByte(u16Modbus_TransmitBuffer[1]);

    break;
  }
  // xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx end of setup ADU frame xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

  u16CRC = 0xffff;

  for (i = 0; i < u8ModbusADUSize; i++)
  {
    u16CRC = util_crc16_update(u16CRC, u8ModbusADU[i]);
  }

  u8ModbusADU[u8ModbusADUSize++] = lowByte(u16CRC);
  u8ModbusADU[u8ModbusADUSize++] = highByte(u16CRC);
  u8ModbusADU[u8ModbusADUSize] = 0;

  Queue_MakeNull(&modbus_master_rx_queue);
  ModbusMaster_CompleteTransmitReQ = 0x00;

  for (i = 0; i < u8ModbusADUSize; i++)
  {
    if (Queue_IsFull(&modbus_master_tx_queue) == 0)
    {
      Queue_EnQueue(&modbus_master_tx_queue, u8ModbusADU[i]);
    }
    else
    {
      // "idle();"
    }
  }

  ModbusMaster_BeginTransmit();

  while (ModbusMaster_CompleteTransmitReQ == 0x00)
    ;

  ModbusMaster_ClearADU();

  u8ModBusStatus = ModBusStatusTransactionSuccess;

  u8ModbusADUSize = 0;

  u32ModBusLoopTimeout = ModbusMaster_GetMillis();

  while (u8ModbusADUSize < 3)
  {

    if (Queue_IsEmpty(&modbus_master_rx_queue))
    {
      // "idle();"
    }
    else
    {
      u8ModbusADU[u8ModbusADUSize++] = Queue_DeQueue(&modbus_master_rx_queue);
    }

    if (u8ModbusADUSize >= 3)
    {
      if (u8ModbusADU[0] != u8ModBusSlaveID)
      {
        u8ModBusStatus = ModBusStatusInvalidSlaveID;
        return u8ModBusStatus;
      }

      if ((u8ModbusADU[1] & 0x7F) != u8ModbusFunction)
      {
        u8ModBusStatus = ModBusStatusInvalidFunction;
        return u8ModBusStatus;
      }

      if (bitRead(u8ModbusADU[1], 7))
      {
        u8ModBusStatus = u8ModbusADU[2];
        return u8ModBusStatus;
      }
    }

    if ((ModbusMaster_GetMillis() - u32ModBusLoopTimeout) > MODBUS_RESPONSE_TIMEOUT && u8ModBusStatus == ModBusStatusTransactionSuccess)
    {
      u8ModBusStatus = ModBusStatusExpiredTimedOut;
      return u8ModBusStatus;
    }
  }

  if (u8ModBusStatus == ModBusStatusTransactionSuccess)
  {

    switch (u8ModbusADU[1])
    {
    case MODBUS_FUNCTION_READ_COILS:
    case MODBUS_FUNCTION_READ_DISCRETE_INPUTS:
    case MODBUS_FUNCTION_READ_HOLDING_REGISTERS:
    case MODBUS_FUNCTION_READ_INPUT_REGISTERS:
    case MODBUS_FUNCTION_READ_WRITE_MUTIPLE_REGISTERS:
      u8BytesLeft = u8ModbusADU[2];
      break;

    case MODBUS_FUNCTION_WRITE_SINGLE_COIL:
    case MODBUS_FUNCTION_WRITE_MUTIPLE_COILS:
    case MODBUS_FUNCTION_WRITE_SINGLE_REGISTER:
    case MODBUS_FUNCTION_WRITE_MUTIPLE_REGISTERS:
      u8BytesLeft = 3;
      break;

    case MODBUS_FUNCTION_MASK_WRITE_REGISTERS:
      u8BytesLeft = 5;
      break;
    }
  }

  u8BytesLeft = u8BytesLeft + 2;

  while (u8BytesLeft)
  {
    if (Queue_IsEmpty(&modbus_master_rx_queue) == 0)
    {
      u8ModbusADU[u8ModbusADUSize++] = Queue_DeQueue(&modbus_master_rx_queue);
      u8BytesLeft--;
    }
    else
    {
      //"idle();"
    }

    if ((ModbusMaster_GetMillis() - u32ModBusLoopTimeout) > MODBUS_RESPONSE_TIMEOUT && u8ModBusStatus == ModBusStatusTransactionSuccess)
    {
      u8ModBusStatus = ModBusStatusExpiredTimedOut;
      return u8ModBusStatus;
    }
  }

  if (u8ModBusStatus == ModBusStatusTransactionSuccess && u8BytesLeft == 0)
  {
    u16CRC = 0xffff;
    for (i = 0; i < (u8ModbusADUSize - 2); i++)
    {
      u16CRC = util_crc16_update(u16CRC, u8ModbusADU[i]);
    }
    if ((lowByte(u16CRC) != u8ModbusADU[u8ModbusADUSize - 2] ||
         highByte(u16CRC) != u8ModbusADU[u8ModbusADUSize - 1]))
    {
      u8ModBusStatus = ModBusStatusInvalidCRC;
      return u8ModBusStatus;
    }
    else
    {
      switch (u8ModbusADU[1])
      {
      case MODBUS_FUNCTION_READ_COILS:
      case MODBUS_FUNCTION_READ_DISCRETE_INPUTS:
        for (i = 0; i < u8ModbusADU[2] >> 1; i++)
        {
          if (i < u8Modbus_ResponseBufferSize)
          {
            u16Modbus_ResponseBuffer[i] = word(u8ModbusADU[(2 * i) + 4], u8ModbusADU[(2 * i) + 3]);
          }
        }
        u8Modbus_TransmitBufferLengh = i;
        if (u8ModbusADU[2] % 2)
        {
          if (i < u8Modbus_ResponseBufferSize)
          {
            u16Modbus_ResponseBuffer[i] = word(0, u8ModbusADU[(2 * i) + 3]);
          }
        }
        u8Modbus_TransmitBufferLengh = i + 1;
        break;
      case MODBUS_FUNCTION_READ_HOLDING_REGISTERS:
      case MODBUS_FUNCTION_READ_INPUT_REGISTERS:
      case MODBUS_FUNCTION_READ_WRITE_MUTIPLE_REGISTERS:
        for (i = 0; i < (u8ModbusADU[2] >> 1); i++)
        {
          if (i < u8Modbus_ResponseBufferSize)
          {
            u16Modbus_ResponseBuffer[i] = word(u8ModbusADU[2 * i + 3], u8ModbusADU[2 * i + 4]);
          }
        }
        u8Modbus_TransmitBufferLengh = i;
        break;
      }
    }
  }

  if ((ModbusMaster_GetMillis() - u32ModBusLoopTimeout) > MODBUS_RESPONSE_TIMEOUT && u8ModBusStatus == ModBusStatusTransactionSuccess)
  {
    u8ModBusStatus = ModBusStatusExpiredTimedOut;
    return u8ModBusStatus;
  }

  return u8ModBusStatus;
}

uint8_t ModbusMaster_readCoils(uint8_t u8SlaveID, uint16_t u16ReadAddress, uint16_t u16BitByteQuantity) // 0x01
{
  u8ModBusSlaveID = u8SlaveID;
  u16ModbusReadAddress = u16ReadAddress;
  u16ModbusReadQuantity = u16BitByteQuantity;
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_READ_COILS);
}

uint8_t ModbusMaster_readDiscreteInputs(uint8_t u8SlaveID, uint16_t u16ReadAddress, uint16_t u16BitByteQuantity) // 0x02
{
  u8ModBusSlaveID = u8SlaveID;
  u16ModbusReadAddress = u16ReadAddress;
  u16ModbusReadQuantity = u16BitByteQuantity;
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_READ_DISCRETE_INPUTS);
}

uint8_t ModbusMaster_ReadHoldingRegisters(uint8_t u8SlaveID, uint16_t u16ReadAddress, uint16_t u16BitByteQuantity) // 0x03
{
  u8ModBusSlaveID = u8SlaveID;
  u16ModbusReadAddress = u16ReadAddress;
  u16ModbusReadQuantity = u16BitByteQuantity;
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_READ_HOLDING_REGISTERS);
}

uint8_t ModbusMaster_ReadInputRegisters(uint8_t u8SlaveID, uint16_t u16ReadAddress, uint16_t u16ReadQuantity) // 0x04
{
  u8ModBusSlaveID = u8SlaveID;
  u16ModbusReadAddress = u16ReadAddress;
  u16ModbusReadQuantity = u16ReadQuantity;
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_READ_INPUT_REGISTERS);
}

uint8_t ModbusMaster_WriteSingleCoil(uint8_t u8SlaveID, uint16_t u16WriteAddress, uint8_t u8State) // 0x05
{
  u8ModBusSlaveID = u8SlaveID;
  u16ModbusWriteAddress = u16WriteAddress;
  if (u8State)
  {
    u16ModbusWriteQuantity = 0xff00;
  }
  else
  {
    u16ModbusWriteQuantity = 0x0000;
  }
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_WRITE_SINGLE_COIL);
}

uint8_t ModbusMaster_WriteSingleRegister(uint8_t u8SlaveID, uint16_t u16WriteAddress, uint16_t u16WriteValue) // 0x06
{
  u8ModBusSlaveID = u8SlaveID;
  u16ModbusWriteAddress = u16WriteAddress;
  u16ModbusWriteQuantity = 0;
  u16Modbus_TransmitBuffer[0] = u16WriteValue;
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_WRITE_SINGLE_REGISTER);
}

uint8_t ModbusMaster_WriteMultipleCoils(uint8_t u8SlaveID, uint16_t u16WriteAddress, uint16_t u16BitByteQuantity) // 0x0F
{
  u8ModBusSlaveID = u8SlaveID;
  u16ModbusWriteAddress = u16WriteAddress;
  u16ModbusWriteQuantity = u16BitByteQuantity;
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_WRITE_MUTIPLE_COILS);
}

uint8_t ModbusMaster_WriteMultipleRegisters(uint8_t u8SlaveID, uint16_t u16WriteAddress, uint16_t u16WriteQuantity) // 0x10
{
  u8ModBusSlaveID = u8SlaveID;
  u16ModbusWriteAddress = u16WriteAddress;
  u16ModbusWriteQuantity = u16WriteQuantity;
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_WRITE_MUTIPLE_REGISTERS);
}

uint8_t ModbusMaster_MaskWriteRegister(uint8_t u8SlaveID, uint16_t u16WriteAddress, uint16_t u16AndMask, uint16_t u16OrMask) // 0x16
{
  u8ModBusSlaveID = u8SlaveID;
  u16ModbusWriteAddress = u16WriteAddress;
  u16Modbus_TransmitBuffer[0] = u16AndMask;
  u16Modbus_TransmitBuffer[1] = u16OrMask;
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_MASK_WRITE_REGISTERS);
}

uint8_t ModbusMaster_ReadWriteMultipleRegisters(uint8_t u8SlaveID, uint16_t u16ReadAddress, uint16_t u16ReadQuantity,
                                                uint16_t u16WriteAddress, uint16_t u16WriteQuantity) // 0x17
{
  u8ModBusSlaveID = u8SlaveID;
  u16ModbusReadAddress = u16ReadAddress;
  u16ModbusReadQuantity = u16ReadQuantity;
  u16ModbusWriteAddress = u16WriteAddress;
  u16ModbusWriteQuantity = u16WriteQuantity;
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_READ_WRITE_MUTIPLE_REGISTERS);
}
