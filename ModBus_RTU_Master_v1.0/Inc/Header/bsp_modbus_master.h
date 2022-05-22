/**
 * @file       bsp_modbus_master.h
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2022-05-22
 * @author     Thuan Le
 * @brief      Board Support Package Modbus Master
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BSP_MODBUS_MASTER_H
#define __BSP_MODBUS_MASTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include "modbus_master.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
void bsp_modbus_master_init(void);
uint8_t bsp_modbus_master_read_input_register(uint8_t slave_id, uint16_t read_addr, uint16_t size);
uint16_t bsp_modbus_master_get_response_buffer(uint8_t index);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __BSP_MODBUS_MASTER_H

/* End of file -------------------------------------------------------- */
