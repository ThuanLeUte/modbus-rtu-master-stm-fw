#include "modbus_master.h"

#define MODBUS_STATUS_TRANSACTION_SUCCESS             (0x00)
#define MODBUS_STATUS_INVALID_SLAVE_ID                (0xE0)
#define MODBUS_STATUS_INVALID_FUNCTION                (0xE1)
#define MODBUS_STATUS_EXPIRED_TIMEOUT                 (0xE2)
#define MODBUS_STATUS_INVALID_CRC                     (0xE3)

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
static uint8_t quantity = 0;
static uint16_t crc16 = 0xFFFF;
static uint8_t byte_left = 0;
static uint32_t modbus_loop_timeout = 0;
static uint8_t modbus_status = MODBUS_STATUS_TRANSACTION_SUCCESS;
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

void modbus_master_init(modbus_master_t *me)
{
  Queue_Initialize(&modbus_master_tx_queue, modbus_master_tx_buffer, sizeof(modbus_master_tx_buffer));
  Queue_Initialize(&modbus_master_rx_queue, modbus_master_rx_buffer, sizeof(modbus_master_rx_buffer));
}

void modbus_master_clear_adu(modbus_master_t *me)
{
  for (uint8_t k = 0; k < 128; k++)
  {
    modbus_adu[k] = 0;
  }
}

uint16_t modbus_master_get_response_buffer(modbus_master_t *me, uint8_t index)
{
  return modbus_response_buffer[index];
}

uint8_t modbus_master_execute_transaction(modbus_master_t *me, uint8_t modbus_function)
{
  modbus_adu_size = 0;
  i = 0;
  quantity = 0;
  crc16 = 0xFFFF;
  byte_left = 0;
  modbus_loop_timeout = 0;
  modbus_status = MODBUS_STATUS_TRANSACTION_SUCCESS;

  // xxxxxxxxxxxxxxxxxxxxxxx setup ADU frame xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //  assemble Modbus Request Application Data Unit
  modbus_adu[modbus_adu_size++] = modbus_slave_id;
  modbus_adu[modbus_adu_size++] = modbus_function;

  switch (modbus_function)
  {
  case MODBUS_FUNCTION_READ_COILS:
  case MODBUS_FUNCTION_READ_DISCRETE_INPUTS:
  case MODBUS_FUNCTION_READ_HOLDING_REGISTERS:
  case MODBUS_FUNCTION_READ_INPUT_REGISTERS:
  case MODBUS_FUNCTION_READ_WRITE_MUTIPLE_REGISTERS:

    modbus_adu[modbus_adu_size++] = HIGH_BYTE(modbus_read_addr);
    modbus_adu[modbus_adu_size++] = LOW_BYTE(modbus_read_addr);
    modbus_adu[modbus_adu_size++] = HIGH_BYTE(modbus_read_quantity);
    modbus_adu[modbus_adu_size++] = LOW_BYTE(modbus_read_quantity);

    break;
  }

  switch (modbus_function)
  {
  case MODBUS_FUNCTION_WRITE_SINGLE_COIL:
  case MODBUS_FUNCTION_MASK_WRITE_REGISTERS:
  case MODBUS_FUNCTION_WRITE_MUTIPLE_COILS:
  case MODBUS_FUNCTION_WRITE_SINGLE_REGISTER:
  case MODBUS_FUNCTION_WRITE_MUTIPLE_REGISTERS:
  case MODBUS_FUNCTION_READ_WRITE_MUTIPLE_REGISTERS:

    modbus_adu[modbus_adu_size++] = HIGH_BYTE(modbus_write_addr);
    modbus_adu[modbus_adu_size++] = LOW_BYTE(modbus_write_addr);

    break;
  }

  switch (modbus_function)
  {
  case MODBUS_FUNCTION_WRITE_SINGLE_COIL:
    modbus_adu[modbus_adu_size++] = HIGH_BYTE(modbus_write_quantity);
    modbus_adu[modbus_adu_size++] = LOW_BYTE(modbus_write_quantity);
    break;

  case MODBUS_FUNCTION_WRITE_SINGLE_REGISTER:
    modbus_adu[modbus_adu_size++] = HIGH_BYTE(modbus_transmit_buffer[0]);
    modbus_adu[modbus_adu_size++] = LOW_BYTE(modbus_transmit_buffer[0]);
    break;

  case MODBUS_FUNCTION_WRITE_MUTIPLE_COILS:
    modbus_adu[modbus_adu_size++] = HIGH_BYTE(modbus_write_quantity);
    modbus_adu[modbus_adu_size++] = LOW_BYTE(modbus_write_quantity);

    if (modbus_write_quantity % 8)
    {
      quantity = (modbus_write_quantity >> 3) + 1;
    }
    else
    {
      quantity = modbus_write_quantity >> 3;
    }

    for (i = 0; i < quantity; i++)
    {

      switch (i % 2)
      {
      case 0:
        modbus_adu[modbus_adu_size++] = LOW_BYTE(modbus_transmit_buffer[i >> 1]);
        break;

      case 1:
        modbus_adu[modbus_adu_size++] = HIGH_BYTE(modbus_transmit_buffer[i >> 1]);
        break;
      }
    }

    break;

  case MODBUS_FUNCTION_WRITE_MUTIPLE_REGISTERS:
  case MODBUS_FUNCTION_READ_WRITE_MUTIPLE_REGISTERS:
    for (i = 0; i < LOW_BYTE(modbus_write_quantity); i++)
    {
      modbus_adu[modbus_adu_size++] = HIGH_BYTE(modbus_transmit_buffer[i]);
      modbus_adu[modbus_adu_size++] = LOW_BYTE(modbus_transmit_buffer[i]);
    }
    break;

  case MODBUS_FUNCTION_MASK_WRITE_REGISTERS:
    modbus_adu[modbus_adu_size++] = HIGH_BYTE(modbus_transmit_buffer[0]);
    modbus_adu[modbus_adu_size++] = LOW_BYTE(modbus_transmit_buffer[0]);
    modbus_adu[modbus_adu_size++] = HIGH_BYTE(modbus_transmit_buffer[1]);
    modbus_adu[modbus_adu_size++] = LOW_BYTE(modbus_transmit_buffer[1]);

    break;
  }
  // xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx end of setup ADU frame xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

  crc16 = 0xFFFF;

  for (i = 0; i < modbus_adu_size; i++)
  {
    crc16 = util_crc16_update(crc16, modbus_adu[i]);
  }

  modbus_adu[modbus_adu_size++] = LOW_BYTE(crc16);
  modbus_adu[modbus_adu_size++] = HIGH_BYTE(crc16);
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

  me->bsp_uart_start_transmit();
  
  while (modbus_complete_transmit_req == 0x00)
    ;

  modbus_master_clear_adu(me);

  modbus_status = MODBUS_STATUS_TRANSACTION_SUCCESS;

  modbus_adu_size = 0;

  modbus_loop_timeout = me->bsp_get_tick();

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
        modbus_status = MODBUS_STATUS_INVALID_SLAVE_ID;
        return modbus_status;
      }

      if ((modbus_adu[1] & 0x7F) != modbus_function)
      {
        modbus_status = MODBUS_STATUS_INVALID_FUNCTION;
        return modbus_status;
      }

      if (BIT_READ(modbus_adu[1], 7))
      {
        modbus_status = modbus_adu[2];
        return modbus_status;
      }
    }

    if ((me->bsp_get_tick() - modbus_loop_timeout) > MODBUS_RESPONSE_TIMEOUT && modbus_status == MODBUS_STATUS_TRANSACTION_SUCCESS)
    {
      modbus_status = MODBUS_STATUS_EXPIRED_TIMEOUT;
      return modbus_status;
    }
  }

  if (modbus_status == MODBUS_STATUS_TRANSACTION_SUCCESS)
  {

    switch (modbus_adu[1])
    {
    case MODBUS_FUNCTION_READ_COILS:
    case MODBUS_FUNCTION_READ_DISCRETE_INPUTS:
    case MODBUS_FUNCTION_READ_HOLDING_REGISTERS:
    case MODBUS_FUNCTION_READ_INPUT_REGISTERS:
    case MODBUS_FUNCTION_READ_WRITE_MUTIPLE_REGISTERS:
      byte_left = modbus_adu[2];
      break;

    case MODBUS_FUNCTION_WRITE_SINGLE_COIL:
    case MODBUS_FUNCTION_WRITE_MUTIPLE_COILS:
    case MODBUS_FUNCTION_WRITE_SINGLE_REGISTER:
    case MODBUS_FUNCTION_WRITE_MUTIPLE_REGISTERS:
      byte_left = 3;
      break;

    case MODBUS_FUNCTION_MASK_WRITE_REGISTERS:
      byte_left = 5;
      break;
    }
  }

  byte_left = byte_left + 2;

  while (byte_left)
  {
    if (Queue_IsEmpty(&modbus_master_rx_queue) == 0)
    {
      modbus_adu[modbus_adu_size++] = Queue_DeQueue(&modbus_master_rx_queue);
      byte_left--;
    }
    else
    {
      //"idle();"
    }

    if ((me->bsp_get_tick() - modbus_loop_timeout) > MODBUS_RESPONSE_TIMEOUT && modbus_status == MODBUS_STATUS_TRANSACTION_SUCCESS)
    {
      modbus_status = MODBUS_STATUS_EXPIRED_TIMEOUT;
      return modbus_status;
    }
  }

  if (modbus_status == MODBUS_STATUS_TRANSACTION_SUCCESS && byte_left == 0)
  {
    crc16 = 0xFFFF;
    for (i = 0; i < (modbus_adu_size - 2); i++)
    {
      crc16 = util_crc16_update(crc16, modbus_adu[i]);
    }
    if ((LOW_BYTE(crc16) != modbus_adu[modbus_adu_size - 2] ||
         HIGH_BYTE(crc16) != modbus_adu[modbus_adu_size - 1]))
    {
      modbus_status = MODBUS_STATUS_INVALID_CRC;
      return modbus_status;
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
            modbus_response_buffer[i] = util_word(modbus_adu[(2 * i) + 4], modbus_adu[(2 * i) + 3]);
          }
        }
        if (modbus_adu[2] % 2)
        {
          if (i < MODEBUS_MASTER_RX_BUFFER_SIZE)
          {
            modbus_response_buffer[i] = util_word(0, modbus_adu[(2 * i) + 3]);
          }
        }
        break;
      case MODBUS_FUNCTION_READ_HOLDING_REGISTERS:
      case MODBUS_FUNCTION_READ_INPUT_REGISTERS:
      case MODBUS_FUNCTION_READ_WRITE_MUTIPLE_REGISTERS:
        for (i = 0; i < (modbus_adu[2] >> 1); i++)
        {
          if (i < MODEBUS_MASTER_RX_BUFFER_SIZE)
          {
            modbus_response_buffer[i] = util_word(modbus_adu[2 * i + 3], modbus_adu[2 * i + 4]);
          }
        }
        break;
      }
    }
  }

  if ((me->bsp_get_tick() - modbus_loop_timeout) > MODBUS_RESPONSE_TIMEOUT && modbus_status == MODBUS_STATUS_TRANSACTION_SUCCESS)
  {
    modbus_status = MODBUS_STATUS_EXPIRED_TIMEOUT;
    return modbus_status;
  }

  return modbus_status;
}

uint8_t modbus_master_read_coils(modbus_master_t *me, uint8_t slave_id, uint16_t read_addr, uint16_t size)
{
  modbus_slave_id = slave_id;
  modbus_read_addr = read_addr;
  modbus_read_quantity = size;
  return modbus_master_execute_transaction(me, MODBUS_FUNCTION_READ_COILS);
}

uint8_t modbus_master_read_discrete_input(modbus_master_t *me, uint8_t slave_id, uint16_t read_addr, uint16_t size)
{
  modbus_slave_id = slave_id;
  modbus_read_addr = read_addr;
  modbus_read_quantity = size;
  return modbus_master_execute_transaction(me, MODBUS_FUNCTION_READ_DISCRETE_INPUTS);
}

uint8_t modbus_master_read_holding_register(modbus_master_t *me, uint8_t slave_id, uint16_t read_addr, uint16_t size)
{
  modbus_slave_id = slave_id;
  modbus_read_addr = read_addr;
  modbus_read_quantity = size;
  return modbus_master_execute_transaction(me, MODBUS_FUNCTION_READ_HOLDING_REGISTERS);
}

uint8_t modbus_master_read_input_register(modbus_master_t *me, uint8_t slave_id, uint16_t read_addr, uint16_t size)
{
  modbus_slave_id = slave_id;
  modbus_read_addr = read_addr;
  modbus_read_quantity = size;
  return modbus_master_execute_transaction(me, MODBUS_FUNCTION_READ_INPUT_REGISTERS);
}

uint8_t modbus_master_write_single_coil(modbus_master_t *me, uint8_t slave_id, uint16_t write_addr, uint8_t state)
{
  modbus_slave_id = slave_id;
  modbus_write_addr = write_addr;
  if (state)
  {
    modbus_write_quantity = 0xff00;
  }
  else
  {
    modbus_write_quantity = 0x0000;
  }
  return modbus_master_execute_transaction(me, MODBUS_FUNCTION_WRITE_SINGLE_COIL);
}

uint8_t modbus_master_write_single_register(modbus_master_t *me, uint8_t slave_id, uint16_t write_addr, uint16_t value)
{
  modbus_slave_id = slave_id;
  modbus_write_addr = write_addr;
  modbus_write_quantity = 0;
  modbus_transmit_buffer[0] = value;
  return modbus_master_execute_transaction(me, MODBUS_FUNCTION_WRITE_SINGLE_REGISTER);
}

uint8_t modbus_master_write_mutiple_coils(modbus_master_t *me, uint8_t slave_id, uint16_t write_addr, uint16_t size)
{
  modbus_slave_id = slave_id;
  modbus_write_addr = write_addr;
  modbus_write_quantity = size;
  return modbus_master_execute_transaction(me, MODBUS_FUNCTION_WRITE_MUTIPLE_COILS);
}

uint8_t modbus_master_write_mutiple_register(modbus_master_t *me, uint8_t slave_id, uint16_t write_addr, uint16_t size)
{
  modbus_slave_id = slave_id;
  modbus_write_addr = write_addr;
  modbus_write_quantity = size;
  return modbus_master_execute_transaction(me, MODBUS_FUNCTION_WRITE_MUTIPLE_REGISTERS);
}

uint8_t modbus_master_mask_write_register(modbus_master_t *me, uint8_t slave_id, uint16_t write_addr, uint16_t and_mask, uint16_t or_mask)
{
  modbus_slave_id = slave_id;
  modbus_write_addr = write_addr;
  modbus_transmit_buffer[0] = and_mask;
  modbus_transmit_buffer[1] = or_mask;
  return modbus_master_execute_transaction(me, MODBUS_FUNCTION_MASK_WRITE_REGISTERS);
}

uint8_t modbus_maste_read_mutiple_registers(modbus_master_t *me, uint8_t slave_id, uint16_t read_addr, uint16_t read_size,
                                                uint16_t write_addr, uint16_t write_size)
{
  modbus_slave_id = slave_id;
  modbus_read_addr = read_addr;
  modbus_read_quantity = read_size;
  modbus_write_addr = write_addr;
  modbus_write_quantity = write_size;
  return modbus_master_execute_transaction(me, MODBUS_FUNCTION_READ_WRITE_MUTIPLE_REGISTERS);
}
