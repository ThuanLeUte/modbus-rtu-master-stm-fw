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

#define MODEBUS_MASTER_TX_BUFFER_SIZE                 (64)
#define MODEBUS_MASTER_RX_BUFFER_SIZE                 (64)

struct_queue_array modbus_master_tx_queue;
struct_queue_array modbus_master_rx_queue;

static uint8_t modbus_master_tx_buffer[64];
static uint8_t modbus_master_rx_buffer[64];

static uint16_t modbus_transmit_buffer[MODEBUS_MASTER_TX_BUFFER_SIZE]; // Data want to write to slave
static uint16_t modbus_response_buffer[MODEBUS_MASTER_RX_BUFFER_SIZE]; // Data response from slave
static uint8_t modbus_transmit_buffer_len = 0;
static uint8_t modbus_response_buffer_len = 0;

volatile uint8_t modbus_complete_transmit_req = 0;
static uint8_t modbus_slave_id;
static uint16_t modbus_read_addr;
static uint16_t modbus_read_quantity;
static uint16_t modbus_write_addr;
static uint16_t modbus_write_quantity;

// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
static uint8_t modbus_adu[128];
static uint8_t modbus_adu_size = 0;
static uint8_t i = 0;
static uint8_t u8Quantity = 0;
static uint16_t u16CRC = 0xffff;
static uint8_t u8BytesLeft = 0;
static uint32_t u32ModBusLoopTimeout = 0;
static uint8_t u8ModBusStatus = ModBusStatusTransactionSuccess;
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

void ModbusMaster_Begin(void)
{
  modbus_transmit_buffer_len = 0;
  modbus_response_buffer_len = 0;
  Queue_Initialize(&modbus_master_tx_queue, modbus_master_tx_buffer, sizeof(modbus_master_tx_buffer));
  Queue_Initialize(&modbus_master_rx_queue, modbus_master_rx_buffer, sizeof(modbus_master_rx_buffer));
}

void ModbusMaster_ClearADU(void)
{
  for (uint8_t k = 0; k < 128; k++)
  {
    modbus_adu[k] = 0;
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
  if (Index < MODEBUS_MASTER_RX_BUFFER_SIZE)
  {
    return modbus_response_buffer[Index];
  }
}

void ModbusMaster_ClearResponseBuffer(void)
{
  uint8_t i;

  for (i = 0; i < MODEBUS_MASTER_RX_BUFFER_SIZE; i++)
  {
    modbus_response_buffer[i] = 0;
  }
}

uint8_t ModbusMaster_SetTransmitBuffer(uint8_t Index, uint16_t Value)
{
  if (Index < MODEBUS_MASTER_TX_BUFFER_SIZE)
  {
    modbus_transmit_buffer[Index] = Value;
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

  for (i = 0; i < MODEBUS_MASTER_TX_BUFFER_SIZE; i++)
  {
    modbus_transmit_buffer[i] = 0;
  }
}

uint8_t ModbusMaster_Execute_Transaction(uint8_t u8ModbusFunction)
{

  modbus_adu_size = 0;
  i = 0;
  u8Quantity = 0;
  u16CRC = 0xffff;
  u8BytesLeft = 0;
  u32ModBusLoopTimeout = 0;
  u8ModBusStatus = ModBusStatusTransactionSuccess;

  // xxxxxxxxxxxxxxxxxxxxxxx setup ADU frame xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //  assemble Modbus Request Application Data Unit
  modbus_adu[modbus_adu_size++] = modbus_slave_id;
  modbus_adu[modbus_adu_size++] = u8ModbusFunction;

  switch (u8ModbusFunction)
  {
  case MODBUS_FUNCTION_READ_COILS:
  case MODBUS_FUNCTION_READ_DISCRETE_INPUTS:
  case MODBUS_FUNCTION_READ_HOLDING_REGISTERS:
  case MODBUS_FUNCTION_READ_INPUT_REGISTERS:
  case MODBUS_FUNCTION_READ_WRITE_MUTIPLE_REGISTERS:

    modbus_adu[modbus_adu_size++] = highByte(modbus_read_addr);
    modbus_adu[modbus_adu_size++] = lowByte(modbus_read_addr);
    modbus_adu[modbus_adu_size++] = highByte(modbus_read_quantity);
    modbus_adu[modbus_adu_size++] = lowByte(modbus_read_quantity);

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

    modbus_adu[modbus_adu_size++] = highByte(modbus_write_addr);
    modbus_adu[modbus_adu_size++] = lowByte(modbus_write_addr);

    break;
  }

  switch (u8ModbusFunction)
  {
  case MODBUS_FUNCTION_WRITE_SINGLE_COIL:
    modbus_adu[modbus_adu_size++] = highByte(modbus_write_quantity);
    modbus_adu[modbus_adu_size++] = lowByte(modbus_write_quantity);
    break;

  case MODBUS_FUNCTION_WRITE_SINGLE_REGISTER:
    modbus_adu[modbus_adu_size++] = highByte(modbus_transmit_buffer[0]);
    modbus_adu[modbus_adu_size++] = lowByte(modbus_transmit_buffer[0]);
    break;

  case MODBUS_FUNCTION_WRITE_MUTIPLE_COILS:
    modbus_adu[modbus_adu_size++] = highByte(modbus_write_quantity);
    modbus_adu[modbus_adu_size++] = lowByte(modbus_write_quantity);

    if (modbus_write_quantity % 8)
    {
      u8Quantity = (modbus_write_quantity >> 3) + 1;
    }
    else
    {
      u8Quantity = modbus_write_quantity >> 3;
    }

    for (i = 0; i < u8Quantity; i++)
    {

      switch (i % 2)
      {
      case 0:
        modbus_adu[modbus_adu_size++] = lowByte(modbus_transmit_buffer[i >> 1]);
        break;

      case 1:
        modbus_adu[modbus_adu_size++] = highByte(modbus_transmit_buffer[i >> 1]);
        break;
      }
    }

    break;

  case MODBUS_FUNCTION_WRITE_MUTIPLE_REGISTERS:
  case MODBUS_FUNCTION_READ_WRITE_MUTIPLE_REGISTERS:
    for (i = 0; i < lowByte(modbus_write_quantity); i++)
    {
      modbus_adu[modbus_adu_size++] = highByte(modbus_transmit_buffer[i]);
      modbus_adu[modbus_adu_size++] = lowByte(modbus_transmit_buffer[i]);
    }
    break;

  case MODBUS_FUNCTION_MASK_WRITE_REGISTERS:
    modbus_adu[modbus_adu_size++] = highByte(modbus_transmit_buffer[0]);
    modbus_adu[modbus_adu_size++] = lowByte(modbus_transmit_buffer[0]);
    modbus_adu[modbus_adu_size++] = highByte(modbus_transmit_buffer[1]);
    modbus_adu[modbus_adu_size++] = lowByte(modbus_transmit_buffer[1]);

    break;
  }
  // xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx end of setup ADU frame xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

  u16CRC = 0xffff;

  for (i = 0; i < modbus_adu_size; i++)
  {
    u16CRC = util_crc16_update(u16CRC, modbus_adu[i]);
  }

  modbus_adu[modbus_adu_size++] = lowByte(u16CRC);
  modbus_adu[modbus_adu_size++] = highByte(u16CRC);
  modbus_adu[modbus_adu_size] = 0;

  Queue_MakeNull(&modbus_master_rx_queue);
  modbus_complete_transmit_req = 0x00;

  for (i = 0; i < modbus_adu_size; i++)
  {
    if (Queue_IsFull(&modbus_master_tx_queue) == 0)
    {
      Queue_EnQueue(&modbus_master_tx_queue, modbus_adu[i]);
    }
    else
    {
      // "idle();"
    }
  }

  ModbusMaster_BeginTransmit();

  while (modbus_complete_transmit_req == 0x00)
    ;

  ModbusMaster_ClearADU();

  u8ModBusStatus = ModBusStatusTransactionSuccess;

  modbus_adu_size = 0;

  u32ModBusLoopTimeout = ModbusMaster_GetMillis();

  while (modbus_adu_size < 3)
  {

    if (Queue_IsEmpty(&modbus_master_rx_queue))
    {
      // "idle();"
    }
    else
    {
      modbus_adu[modbus_adu_size++] = Queue_DeQueue(&modbus_master_rx_queue);
    }

    if (modbus_adu_size >= 3)
    {
      if (modbus_adu[0] != modbus_slave_id)
      {
        u8ModBusStatus = ModBusStatusInvalidSlaveID;
        return u8ModBusStatus;
      }

      if ((modbus_adu[1] & 0x7F) != u8ModbusFunction)
      {
        u8ModBusStatus = ModBusStatusInvalidFunction;
        return u8ModBusStatus;
      }

      if (bitRead(modbus_adu[1], 7))
      {
        u8ModBusStatus = modbus_adu[2];
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

    switch (modbus_adu[1])
    {
    case MODBUS_FUNCTION_READ_COILS:
    case MODBUS_FUNCTION_READ_DISCRETE_INPUTS:
    case MODBUS_FUNCTION_READ_HOLDING_REGISTERS:
    case MODBUS_FUNCTION_READ_INPUT_REGISTERS:
    case MODBUS_FUNCTION_READ_WRITE_MUTIPLE_REGISTERS:
      u8BytesLeft = modbus_adu[2];
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
      modbus_adu[modbus_adu_size++] = Queue_DeQueue(&modbus_master_rx_queue);
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
    for (i = 0; i < (modbus_adu_size - 2); i++)
    {
      u16CRC = util_crc16_update(u16CRC, modbus_adu[i]);
    }
    if ((lowByte(u16CRC) != modbus_adu[modbus_adu_size - 2] ||
         highByte(u16CRC) != modbus_adu[modbus_adu_size - 1]))
    {
      u8ModBusStatus = ModBusStatusInvalidCRC;
      return u8ModBusStatus;
    }
    else
    {
      switch (modbus_adu[1])
      {
      case MODBUS_FUNCTION_READ_COILS:
      case MODBUS_FUNCTION_READ_DISCRETE_INPUTS:
        for (i = 0; i < modbus_adu[2] >> 1; i++)
        {
          if (i < MODEBUS_MASTER_RX_BUFFER_SIZE)
          {
            modbus_response_buffer[i] = word(modbus_adu[(2 * i) + 4], modbus_adu[(2 * i) + 3]);
          }
        }
        modbus_transmit_buffer_len = i;
        if (modbus_adu[2] % 2)
        {
          if (i < MODEBUS_MASTER_RX_BUFFER_SIZE)
          {
            modbus_response_buffer[i] = word(0, modbus_adu[(2 * i) + 3]);
          }
        }
        modbus_transmit_buffer_len = i + 1;
        break;
      case MODBUS_FUNCTION_READ_HOLDING_REGISTERS:
      case MODBUS_FUNCTION_READ_INPUT_REGISTERS:
      case MODBUS_FUNCTION_READ_WRITE_MUTIPLE_REGISTERS:
        for (i = 0; i < (modbus_adu[2] >> 1); i++)
        {
          if (i < MODEBUS_MASTER_RX_BUFFER_SIZE)
          {
            modbus_response_buffer[i] = word(modbus_adu[2 * i + 3], modbus_adu[2 * i + 4]);
          }
        }
        modbus_transmit_buffer_len = i;
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
  modbus_slave_id = u8SlaveID;
  modbus_read_addr = u16ReadAddress;
  modbus_read_quantity = u16BitByteQuantity;
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_READ_COILS);
}

uint8_t ModbusMaster_readDiscreteInputs(uint8_t u8SlaveID, uint16_t u16ReadAddress, uint16_t u16BitByteQuantity) // 0x02
{
  modbus_slave_id = u8SlaveID;
  modbus_read_addr = u16ReadAddress;
  modbus_read_quantity = u16BitByteQuantity;
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_READ_DISCRETE_INPUTS);
}

uint8_t ModbusMaster_ReadHoldingRegisters(uint8_t u8SlaveID, uint16_t u16ReadAddress, uint16_t u16BitByteQuantity) // 0x03
{
  modbus_slave_id = u8SlaveID;
  modbus_read_addr = u16ReadAddress;
  modbus_read_quantity = u16BitByteQuantity;
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_READ_HOLDING_REGISTERS);
}

uint8_t ModbusMaster_ReadInputRegisters(uint8_t u8SlaveID, uint16_t u16ReadAddress, uint16_t u16ReadQuantity) // 0x04
{
  modbus_slave_id = u8SlaveID;
  modbus_read_addr = u16ReadAddress;
  modbus_read_quantity = u16ReadQuantity;
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_READ_INPUT_REGISTERS);
}

uint8_t ModbusMaster_WriteSingleCoil(uint8_t u8SlaveID, uint16_t u16WriteAddress, uint8_t u8State) // 0x05
{
  modbus_slave_id = u8SlaveID;
  modbus_write_addr = u16WriteAddress;
  if (u8State)
  {
    modbus_write_quantity = 0xff00;
  }
  else
  {
    modbus_write_quantity = 0x0000;
  }
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_WRITE_SINGLE_COIL);
}

uint8_t ModbusMaster_WriteSingleRegister(uint8_t u8SlaveID, uint16_t u16WriteAddress, uint16_t u16WriteValue) // 0x06
{
  modbus_slave_id = u8SlaveID;
  modbus_write_addr = u16WriteAddress;
  modbus_write_quantity = 0;
  modbus_transmit_buffer[0] = u16WriteValue;
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_WRITE_SINGLE_REGISTER);
}

uint8_t ModbusMaster_WriteMultipleCoils(uint8_t u8SlaveID, uint16_t u16WriteAddress, uint16_t u16BitByteQuantity) // 0x0F
{
  modbus_slave_id = u8SlaveID;
  modbus_write_addr = u16WriteAddress;
  modbus_write_quantity = u16BitByteQuantity;
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_WRITE_MUTIPLE_COILS);
}

uint8_t ModbusMaster_WriteMultipleRegisters(uint8_t u8SlaveID, uint16_t u16WriteAddress, uint16_t u16WriteQuantity) // 0x10
{
  modbus_slave_id = u8SlaveID;
  modbus_write_addr = u16WriteAddress;
  modbus_write_quantity = u16WriteQuantity;
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_WRITE_MUTIPLE_REGISTERS);
}

uint8_t ModbusMaster_MaskWriteRegister(uint8_t u8SlaveID, uint16_t u16WriteAddress, uint16_t u16AndMask, uint16_t u16OrMask) // 0x16
{
  modbus_slave_id = u8SlaveID;
  modbus_write_addr = u16WriteAddress;
  modbus_transmit_buffer[0] = u16AndMask;
  modbus_transmit_buffer[1] = u16OrMask;
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_MASK_WRITE_REGISTERS);
}

uint8_t ModbusMaster_ReadWriteMultipleRegisters(uint8_t u8SlaveID, uint16_t u16ReadAddress, uint16_t u16ReadQuantity,
                                                uint16_t u16WriteAddress, uint16_t u16WriteQuantity) // 0x17
{
  modbus_slave_id = u8SlaveID;
  modbus_read_addr = u16ReadAddress;
  modbus_read_quantity = u16ReadQuantity;
  modbus_write_addr = u16WriteAddress;
  modbus_write_quantity = u16WriteQuantity;
  return ModbusMaster_Execute_Transaction(MODBUS_FUNCTION_READ_WRITE_MUTIPLE_REGISTERS);
}
