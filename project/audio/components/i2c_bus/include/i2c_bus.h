#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t i2c_bus_write_reg(uint8_t slave_addr, uint8_t reg_addr, uint8_t data);

esp_err_t i2c_bus_read_reg(uint8_t slave_addr, uint8_t reg_addr, uint8_t *pdata);

esp_err_t i2c_bus_read_data(uint8_t slave_addr, uint8_t *pdata, int size);

esp_err_t i2c_bus_init();

#ifdef __cplusplus
}
#endif
