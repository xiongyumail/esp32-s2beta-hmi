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

/*C Includes*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* I2C Includes */
#include "driver/i2c.h"

/* FT5x06 Include */
#include "ft5x06.h"

#define WRITE_BIT  I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN   0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0               /*!< I2C ack value */
#define NACK_VAL   0x1               /*!< I2C nack value */

static esp_err_t ft5x06_read(uint8_t start_addr, uint8_t read_num, uint8_t *data_buf)
{
    esp_err_t ret = ESP_FAIL;

    if (data_buf != NULL) {
        i2c_cmd_handle_t cmd = NULL;
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (FT5x06_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, start_addr, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(FT5x06_I2C_BUS, cmd, FT5x06_TIMEOUT / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if(ret != ESP_OK) {
            return ESP_FAIL;
        }
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (FT5x06_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
        if(read_num > 1) {
            i2c_master_read(cmd, data_buf, read_num-1, ACK_VAL);
        }
        i2c_master_read_byte(cmd, &data_buf[read_num-1], NACK_VAL);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(FT5x06_I2C_BUS, cmd, FT5x06_TIMEOUT / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
    }
    return ret;
}

static esp_err_t ft5x06_write(uint8_t start_addr, uint8_t write_num, uint8_t *data_buf)
{
    esp_err_t ret = ESP_FAIL;
    if (data_buf != NULL) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (FT5x06_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, start_addr, ACK_CHECK_EN);
        i2c_master_write(cmd, data_buf, write_num, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(FT5x06_I2C_BUS, cmd, FT5x06_TIMEOUT / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
    }
    return ret;
}

static uint16_t read_word(uint8_t reg)
{
    uint8_t data[2];
    uint16_t result;
    ft5x06_read(reg, 2, &data);
    result = data[0] << 8 | data[1];
    return result;
}

static void write_reg(uint8_t reg, uint8_t val)
{
    ft5x06_write(reg, 1, &val);
}

static uint8_t read_byte(uint8_t reg)
{
    uint8_t data;
    ft5x06_read(reg, 1, &data);
    return data;
}

/*Function pointer to read data. Return 'true' if there is still data to be read (buffered)*/
bool ft5x06_pos_read(uint16_t *xpos, uint16_t *ypos)
{
    uint16_t temp;
    // Only take a reading if we are touched.
    if ((read_byte(FT5x06_TOUCH_POINTS) & 0x07)) {
        /* Get the X, Y, values */
        *xpos = read_word(FT5x06_TOUCH1_XH) & 0x0fff;
        *ypos = read_word(FT5x06_TOUCH1_YH);
        temp = *xpos;
        *xpos = *ypos;
        *ypos = SCREEN_YSIZE - 1 - temp;
        return true;
    }
    return false;
}

/* Input device interface */
/* Initialize your touchpad */
void ft5x06_init()
{
    // Init default values. (From NHD-3.5-320240MF-ATXL-CTP-1 datasheet)
    // Valid touching detect threshold
    write_reg(FT5x06_ID_G_THGROUP, 0x16);

    // valid touching peak detect threshold
    write_reg(FT5x06_ID_G_THPEAK, 0x3C);

    // Touch focus threshold
    write_reg(FT5x06_ID_G_THCAL, 0xE9);

    // threshold when there is surface water
    write_reg(FT5x06_ID_G_THWATER, 0x01);

    // threshold of temperature compensation
    write_reg(FT5x06_ID_G_THTEMP, 0x01);

    // Touch difference threshold
    write_reg(FT5x06_ID_G_THDIFF, 0xA0);

    // Delay to enter 'Monitor' status (s)
    write_reg(FT5x06_ID_G_TIME_ENTER_MONITOR, 0x0A);

    // Period of 'Active' status (ms)
    write_reg(FT5x06_ID_G_PERIODACTIVE, 0x06);

    // Timer to enter 'idle' when in 'Monitor' (ms)
    write_reg(FT5x06_ID_G_PERIODMONITOR, 0x28);

}
