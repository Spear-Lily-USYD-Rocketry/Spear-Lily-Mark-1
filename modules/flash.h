#ifndef FLASH_H_
#define FLASH_H_
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "clock_config.h"
#include "fsl_flash.h"
#include "peripheral_intf.h"

void error_trap();
void init_flash(flash_config_t *);
void clear_flash(flash_config_t *, uint32_t );
void write_data(flash_config_t *s_flashDriver, uint32_t write_base_address, uint64_t data_counter, sensor_data_t *data);
void update_data_counter(flash_config_t *s_flashDriver, uint32_t write_base_address, uint64_t data_counter);

#endif /* FLASH_H_ */
