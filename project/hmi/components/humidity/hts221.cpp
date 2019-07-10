// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <stdio.h>
#include "esp_log.h"
#include "hts221.hpp"

#include "I2Cbus.hpp"

typedef struct {
    I2C_t bus;
    uint16_t dev_addr;
} hts221_dev_t;

esp_err_t iot_hts221_write_byte(hts221_handle_t sensor, uint8_t reg_addr, uint8_t data)
{
    hts221_dev_t* sens = (hts221_dev_t*) sensor;
    esp_err_t ret = sens->bus.writeByte((uint8_t)sens->dev_addr, reg_addr, data, -1);
	return ret;
}

esp_err_t iot_hts221_write(hts221_handle_t sensor, uint8_t reg_start_addr, uint8_t reg_num, uint8_t *data_buf)
{
    uint32_t i = 0;
    if (data_buf != NULL) {
        for(i=0; i<reg_num; i++) {
            iot_hts221_write_byte(sensor, reg_start_addr+i, data_buf[i]);
        }
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t iot_hts221_read_byte(hts221_handle_t sensor, uint8_t reg, uint8_t *data)
{
    hts221_dev_t* sens = (hts221_dev_t*) sensor;
    esp_err_t ret = sens->bus.readByte((uint8_t)sens->dev_addr, reg, data, -1);
    return ret;
}

esp_err_t iot_hts221_read(hts221_handle_t sensor, uint8_t reg_start_addr, uint8_t reg_num, uint8_t *data_buf)
{
    uint32_t i = 0;
    uint8_t data_t = 0;
    if (data_buf != NULL) {
        for(i=0; i<reg_num; i++){
            iot_hts221_read_byte(sensor, reg_start_addr+i, &data_t);
            data_buf[i] = data_t;
        }
        return ESP_OK;
    } 
    return ESP_FAIL;  
}

esp_err_t iot_hts221_get_deviceid(hts221_handle_t sensor, uint8_t* deviceid)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_hts221_read_byte(sensor, HTS221_WHO_AM_I_REG, &tmp);
    *deviceid = tmp;
    return ret;
}

esp_err_t iot_hts221_set_config(hts221_handle_t sensor, hts221_config_t* hts221_config)
{
    uint8_t buffer[3];
    iot_hts221_read(sensor, HTS221_AV_CONF_REG, 1, buffer);
    buffer[0] &= ~(HTS221_AVGH_MASK | HTS221_AVGT_MASK);
    buffer[0] |= (uint8_t)hts221_config->avg_h;
    buffer[0] |= (uint8_t)hts221_config->avg_t;
    iot_hts221_write(sensor, HTS221_AV_CONF_REG, 1, buffer);

    iot_hts221_read(sensor, HTS221_CTRL_REG1, 3, buffer);
    buffer[0] &= ~(HTS221_BDU_MASK | HTS221_ODR_MASK);
    buffer[0] |= (uint8_t)hts221_config->odr;
    buffer[0] |= ((uint8_t)hts221_config->bdu_status) << HTS221_BDU_BIT;
    buffer[1] &= ~HTS221_HEATER_BIT;
    buffer[1] |= ((uint8_t)hts221_config->heater_status) << HTS221_HEATER_BIT;
    buffer[2] &= ~(HTS221_DRDY_H_L_MASK | HTS221_PP_OD_MASK | HTS221_DRDY_MASK);
    buffer[2] |= ((uint8_t)hts221_config->irq_level) << HTS221_DRDY_H_L_BIT;
    buffer[2] |= (uint8_t)hts221_config->irq_output_type;
    buffer[2] |= ((uint8_t)hts221_config->irq_enable) << HTS221_DRDY_BIT;
    iot_hts221_write(sensor, HTS221_CTRL_REG1, 3, buffer);
    return ESP_OK;
}

esp_err_t iot_hts221_get_config(hts221_handle_t sensor, hts221_config_t* hts221_config)
{
    uint8_t buffer[3];
    iot_hts221_read(sensor, HTS221_AV_CONF_REG, 1, buffer);
    hts221_config->avg_h = (hts221_avgh_t)(buffer[0] & HTS221_AVGH_MASK);
    hts221_config->avg_t = (hts221_avgt_t)(buffer[0] & HTS221_AVGT_MASK);

    iot_hts221_read(sensor, HTS221_CTRL_REG1, 3, buffer);
    hts221_config->odr = (hts221_odr_t)(buffer[0] & HTS221_ODR_MASK);
    hts221_config->bdu_status = (hts221_state_t)((buffer[0] & HTS221_BDU_MASK) >> HTS221_BDU_BIT);
    hts221_config->heater_status = (hts221_state_t)((buffer[1] & HTS221_HEATER_MASK) >> HTS221_HEATER_BIT);
    hts221_config->irq_level = (hts221_drdylevel_t)(buffer[2] & HTS221_DRDY_H_L_MASK);
    hts221_config->irq_output_type = (hts221_outputtype_t)(buffer[2] & HTS221_PP_OD_MASK);
    hts221_config->irq_enable = (hts221_state_t)((buffer[2] & HTS221_DRDY_MASK) >> HTS221_DRDY_BIT);
    return ESP_OK;
}

esp_err_t iot_hts221_set_activate(hts221_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_hts221_read_byte(sensor, HTS221_CTRL_REG1, &tmp);
    if(ret == ESP_FAIL){
        return ret;
    }
    tmp |= HTS221_PD_MASK;
    ret = iot_hts221_write_byte(sensor, HTS221_CTRL_REG1, tmp);
    return ret;
}

esp_err_t iot_hts221_set_powerdown(hts221_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_hts221_read_byte(sensor, HTS221_CTRL_REG1, &tmp);
    if(ret == ESP_FAIL){
        return ret;
    }
    tmp &= ~HTS221_PD_MASK;
    ret = iot_hts221_write_byte(sensor, HTS221_CTRL_REG1, tmp);
    return ret;
}

esp_err_t iot_hts221_set_odr(hts221_handle_t sensor, hts221_odr_t odr)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_hts221_read_byte(sensor, HTS221_CTRL_REG1, &tmp);
    if(ret == ESP_FAIL){
        return ret;
    }
    tmp &= ~HTS221_ODR_MASK;
    tmp |= (uint8_t)odr;
    ret = iot_hts221_write_byte(sensor, HTS221_CTRL_REG1, tmp);
    return ret;
}

esp_err_t iot_hts221_set_avgh(hts221_handle_t sensor, hts221_avgh_t avgh)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_hts221_read_byte(sensor, HTS221_AV_CONF_REG, &tmp);
    if(ret == ESP_FAIL){
        return ret;
    }
    tmp &= ~HTS221_AVGH_MASK;
    tmp |= (uint8_t)avgh;
    ret = iot_hts221_write_byte(sensor, HTS221_AV_CONF_REG, tmp);
    return ret;
}

esp_err_t iot_hts221_set_avgt(hts221_handle_t sensor, hts221_avgt_t avgt)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_hts221_read_byte(sensor, HTS221_AV_CONF_REG, &tmp);
    if(ret == ESP_FAIL){
        return ret;
    }
    tmp &= ~HTS221_AVGT_MASK;
    tmp |= (uint8_t)avgt;
    ret = iot_hts221_write_byte(sensor, HTS221_AV_CONF_REG, tmp);
    return ret;
}

esp_err_t iot_hts221_set_bdumode(hts221_handle_t sensor, hts221_state_t status)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_hts221_read_byte(sensor, HTS221_CTRL_REG1, &tmp);
    if(ret == ESP_FAIL){
        return ret;
    }
    tmp &= ~HTS221_BDU_MASK;
    tmp |= ((uint8_t)status) << HTS221_BDU_BIT;
    ret = iot_hts221_write_byte(sensor, HTS221_CTRL_REG1, tmp);
    return ret;
}

esp_err_t iot_hts221_memory_boot(hts221_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_hts221_read_byte(sensor, HTS221_CTRL_REG2, &tmp);
    if(ret == ESP_FAIL){
        return ret;
    }
    tmp |= HTS221_BOOT_MASK;
    ret = iot_hts221_write_byte(sensor, HTS221_CTRL_REG2, tmp);
    return ret;
}

esp_err_t iot_hts221_set_heaterstate(hts221_handle_t sensor, hts221_state_t status)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_hts221_read_byte(sensor, HTS221_CTRL_REG2, &tmp);
    if(ret == ESP_FAIL){
        return ret;
    }
    tmp &= ~HTS221_HEATER_MASK;
    tmp |= ((uint8_t)status) << HTS221_HEATER_BIT;
    ret = iot_hts221_write_byte(sensor, HTS221_CTRL_REG2, tmp);
    return ret;
}

esp_err_t iot_hts221_start_oneshot(hts221_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_hts221_read_byte(sensor, HTS221_CTRL_REG2, &tmp);
    if(ret == ESP_FAIL){
        return ret;
    }
    tmp |= HTS221_ONE_SHOT_MASK;
    ret = iot_hts221_write_byte(sensor, HTS221_CTRL_REG2, tmp);
    return ret;
}

esp_err_t iot_hts221_set_irq_activelevel(hts221_handle_t sensor, hts221_drdylevel_t value)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_hts221_read_byte(sensor, HTS221_CTRL_REG3, &tmp);
    if(ret == ESP_FAIL){
        return ret;
    }
    tmp &= ~HTS221_DRDY_H_L_MASK;
    tmp |= (uint8_t)value;
    ret = iot_hts221_write_byte(sensor, HTS221_CTRL_REG3, tmp);
    return ret;
}

esp_err_t iot_hts221_set_irq_outputtype(hts221_handle_t sensor, hts221_outputtype_t value)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_hts221_read_byte(sensor, HTS221_CTRL_REG3, &tmp);
    if(ret == ESP_FAIL){
        return ret;
    }
    tmp &= ~HTS221_PP_OD_MASK;
    tmp |= (uint8_t)value;
    ret = iot_hts221_write_byte(sensor, HTS221_CTRL_REG3, tmp);
    return ret;
}

esp_err_t iot_hts221_set_irq_enable(hts221_handle_t sensor, hts221_state_t status)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = iot_hts221_read_byte(sensor, HTS221_CTRL_REG3, &tmp);
    if(ret == ESP_FAIL){
        return ret;
    }
    tmp &= ~HTS221_DRDY_MASK;
    tmp |= ((uint8_t)status) << HTS221_DRDY_BIT;
    ret = iot_hts221_write_byte(sensor, HTS221_CTRL_REG3, tmp);
    return ret;
}

esp_err_t iot_hts221_get_raw_humidity(hts221_handle_t sensor, int16_t *humidity)
{
    esp_err_t ret;
    uint8_t buffer[2];
    ret = iot_hts221_read(sensor, HTS221_HR_OUT_L_REG, 2, buffer);
    *humidity = (int16_t)((((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0]);
    return ret;
}

esp_err_t iot_hts221_get_humidity(hts221_handle_t sensor, int16_t *humidity)
{
    int16_t h0_t0_out, h1_t0_out, h_t_out;
    int16_t h0_rh, h1_rh;
    uint8_t buffer[2];
    int32_t tmp_32;
    
    iot_hts221_read(sensor, HTS221_H0_RH_X2, 2, buffer);
    h0_rh = buffer[0] >> 1;
    h1_rh = buffer[1] >> 1;
    
    iot_hts221_read(sensor, HTS221_H0_T0_OUT_L, 2, buffer);
    h0_t0_out = (int16_t)(((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

    iot_hts221_read(sensor, HTS221_H1_T0_OUT_L, 2, buffer);
    h1_t0_out = (int16_t)(((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];
    
    iot_hts221_read(sensor, HTS221_HR_OUT_L_REG, 2, buffer);
    h_t_out = (int16_t)(((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];
    
    tmp_32 = ((int32_t)(h_t_out - h0_t0_out)) * ((int32_t)(h1_rh - h0_rh) * 10);
    if (h1_t0_out - h0_t0_out == 0) {
        return ESP_FAIL;
    }
    *humidity = tmp_32/(int32_t)(h1_t0_out - h0_t0_out)  + h0_rh * 10;
    if(*humidity > 1000) {
        *humidity = 1000;
    }
    return ESP_OK;
}

esp_err_t iot_hts221_get_raw_temperature(hts221_handle_t sensor, int16_t *temperature)
{
    esp_err_t ret;
    uint8_t buffer[2];
    ret = iot_hts221_read(sensor, HTS221_TEMP_OUT_L_REG, 2, buffer);
    *temperature = (int16_t)((((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0]);
    return ret;
}

esp_err_t iot_hts221_get_temperature(hts221_handle_t sensor, int16_t *temperature)
{
    int16_t t0_out, t1_out, t_out, t0_degc_x8_u16, t1_degc_x8_u16;
    int16_t t0_degc, t1_degc;
    uint8_t buffer[4], tmp_8;
    int32_t tmp_32;
    iot_hts221_read(sensor, HTS221_T0_DEGC_X8, 2, buffer);
    iot_hts221_read(sensor, HTS221_T0_T1_DEGC_H2, 1, &tmp_8);
    t0_degc_x8_u16 = (((uint16_t)(tmp_8 & 0x03)) << 8) | ((uint16_t)buffer[0]);  
    t1_degc_x8_u16 = (((uint16_t)(tmp_8 & 0x0C)) << 6) | ((uint16_t)buffer[1]);
    t0_degc = t0_degc_x8_u16 >> 3;
    t1_degc = t1_degc_x8_u16 >> 3;

    iot_hts221_read(sensor, HTS221_T0_OUT_L, 4, buffer);
    t0_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];  
    t1_out = (((uint16_t)buffer[3]) << 8) | (uint16_t)buffer[2];

    iot_hts221_read(sensor, HTS221_TEMP_OUT_L_REG, 2, buffer);
    t_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0]; 

    tmp_32 = ((int32_t)(t_out - t0_out)) * ((int32_t)(t1_degc - t0_degc) * 10);
    if ((t1_out - t0_out) == 0) {
        printf("temp error\r\n");
        return ESP_FAIL;
    }
    *temperature = tmp_32 / (int32_t)(t1_out - t0_out) + t0_degc * 10;
    return ESP_OK;
}

hts221_handle_t iot_hts221_create()
{
    hts221_dev_t* sensor = (hts221_dev_t*) calloc(1, sizeof(hts221_dev_t));
    sensor->bus = getI2C(I2C_NUM_0);
    sensor->dev_addr = HTS221_I2C_ADDRESS;
    hts221_config_t hts221_config;
    iot_hts221_get_config(sensor, &hts221_config);
    hts221_config.avg_h = HTS221_AVGH_32;
    hts221_config.avg_t = HTS221_AVGT_16;
    hts221_config.odr = HTS221_ODR_12_5HZ;
    hts221_config.bdu_status = HTS221_DISABLE;
    hts221_config.heater_status = HTS221_DISABLE;
    iot_hts221_set_config(sensor, &hts221_config);
    iot_hts221_set_activate(sensor);
    return (hts221_handle_t) sensor;
}

esp_err_t iot_hts221_delete(hts221_handle_t sensor)
{
    hts221_dev_t* sens = (hts221_dev_t*) sensor;
    free(sens);
    return ESP_OK;
}

// CHts221::CHts221(I2C_t p_i2c_bus, uint8_t addr)
// {
//     bus = p_i2c_bus;
//     m_sensor_handle = iot_hts221_create(bus, addr);
//     hts221_config_t hts221_config;
//     iot_hts221_get_config(m_sensor_handle, &hts221_config);
//     hts221_config.avg_h = HTS221_AVGH_32;
//     hts221_config.avg_t = HTS221_AVGT_16;
//     hts221_config.odr = HTS221_ODR_1HZ;
//     hts221_config.bdu_status = HTS221_DISABLE;
//     hts221_config.heater_status = HTS221_DISABLE;
//     iot_hts221_set_config(m_sensor_handle, &hts221_config);
//     iot_hts221_set_activate(m_sensor_handle);
// }

// CHts221::~CHts221()
// {
//     iot_hts221_delete(m_sensor_handle, false);
//     m_sensor_handle = NULL;
// }

// float CHts221::read_temperature()
// {
//     int16_t temperature;
//     iot_hts221_get_temperature(m_sensor_handle, &temperature);
//     return (float) temperature / 10;
// }

// float CHts221::read_humidity()
// {
//     int16_t humidity;
//     iot_hts221_get_humidity(m_sensor_handle, &humidity);
//     return (float) humidity / 10;
// }

// uint8_t CHts221::id()
// {
//     uint8_t id;
//     iot_hts221_get_deviceid(m_sensor_handle, &id);
//     return id;
// }