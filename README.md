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
