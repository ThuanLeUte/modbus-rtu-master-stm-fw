# How to port the Modbus RTU Master Library

1. File structure

   Source file:

   - bsp.c : Board Support Package
   - modbus_master.c : Mobus RTU Master library (Low Level)
   - bsp_modbus_master.c: Board Support Package for Modbus RTU master (High Level)
   - queue_circule_array.c: Queue for handle Mobus data

   Header file: 

   - bsp.h, bsp_modbus_master.h, modbus_master.h, queue_circule_array.h, util.h

2. Example:

   Check at the main.c file

3. Example 

```
void send_single_register(void)
{
	bsp_rs485_enable_receive(false);
	bsp_rs485_enable_transmit(true);
	
	// slave_id: 0x01
    // write_addr: 0x0000
    // value: 14
	if (bsp_modbus_master_write_single_register(0x01, 0x0000, 14) == 0x00)
  	{
 		// Write success
  	}
  	else
  	{
    	// Write error
  	}
}
```

```
void send_multiple_register(void)
{
	bsp_rs485_enable_receive(false);
	bsp_rs485_enable_transmit(true);
	
	// Prepare transmit data
	bsp_modbus_master_set_transmit_buffer(0, 150);
	bsp_modbus_master_set_transmit_buffer(1, 200);
	bsp_modbus_master_set_transmit_buffer(2, 300);
		
	
    // slave_id: 0x01
    // write_addr: 0x0000
    // size: 3
	if (bsp_modbus_master_write_mutiple_register(0x01, 0x0000, 3) == 0x00)
  	{
 		// Write success
  	}
  	else
  	{
    	// Write error
  	}
}
```

