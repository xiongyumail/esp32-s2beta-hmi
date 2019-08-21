/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * OV2640 driver.
 *
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "sccb.h"
#include "ov2640.h"
#include "ov2640_regs.h"
#include "ov2640_settings.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char* TAG = "ov2640";
#endif

static volatile ov2640_bank_t reg_bank = BANK_MAX;
static int set_bank(sensor_t *sensor, ov2640_bank_t bank)
{
    int res = 0;
    if (bank != reg_bank) {
        reg_bank = bank;
        res = SCCB_Write(sensor->slv_addr, BANK_SEL, bank);
    }
    return res;
}

static int write_regs(sensor_t *sensor, const uint8_t (*regs)[2])
{
    int i=0, res = 0;
    while (regs[i][0]) {
        if (regs[i][0] == BANK_SEL) {
            res = set_bank(sensor, regs[i][1]);
        } else {
            res = SCCB_Write(sensor->slv_addr, regs[i][0], regs[i][1]);
        }
        if (res) {
            return res;
        }
        i++;
    }
    return res;
}

static int write_reg(sensor_t *sensor, ov2640_bank_t bank, uint8_t reg, uint8_t value)
{
    int ret = set_bank(sensor, bank);
    if(!ret) {
        ret = SCCB_Write(sensor->slv_addr, reg, value);
    }
    return ret;
}

static int set_reg_bits(sensor_t *sensor, uint8_t bank, uint8_t reg, uint8_t offset, uint8_t mask, uint8_t value)
{
    int ret = 0;
    uint8_t c_value, new_value;

    ret = set_bank(sensor, bank);
    if(ret) {
        return ret;
    }
    c_value = SCCB_Read(sensor->slv_addr, reg);
    new_value = (c_value & ~(mask << offset)) | ((value & mask) << offset);
    ret = SCCB_Write(sensor->slv_addr, reg, new_value);
    return ret;
}

static int read_reg(sensor_t *sensor, ov2640_bank_t bank, uint8_t reg)
{
    if(set_bank(sensor, bank)){
        return 0;
    }
    return SCCB_Read(sensor->slv_addr, reg);
}

static uint8_t get_reg_bits(sensor_t *sensor, uint8_t bank, uint8_t reg, uint8_t offset, uint8_t mask)
{
    return (read_reg(sensor, bank, reg) >> offset) & mask;
}

static int write_reg_bits(sensor_t *sensor, uint8_t bank, uint8_t reg, uint8_t mask, int enable)
{
    return set_reg_bits(sensor, bank, reg, 0, mask, enable?mask:0);
}

#define WRITE_REGS_OR_RETURN(regs) ret = write_regs(sensor, regs); if(ret){return ret;}
#define WRITE_REG_OR_RETURN(bank, reg, val) ret = write_reg(sensor, bank, reg, val); if(ret){return ret;}
#define SET_REG_BITS_OR_RETURN(bank, reg, offset, mask, val) ret = set_reg_bits(sensor, bank, reg, offset, mask, val); if(ret){return ret;}

static int reset(sensor_t *sensor)
{
    int ret = 0;
    WRITE_REG_OR_RETURN(BANK_SENSOR, COM7, COM7_SRST);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    WRITE_REGS_OR_RETURN(ov2640_settings_cif);
    return ret;
}

static int set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
    int ret = 0;
    sensor->pixformat = pixformat;
    switch (pixformat) {
    case PIXFORMAT_RGB565:
    case PIXFORMAT_RGB888:
        WRITE_REGS_OR_RETURN(ov2640_settings_rgb565);
        break;
    case PIXFORMAT_YUV422:
    case PIXFORMAT_GRAYSCALE:
        WRITE_REGS_OR_RETURN(ov2640_settings_yuv422);
        break;
    case PIXFORMAT_JPEG:
        WRITE_REGS_OR_RETURN(ov2640_settings_jpeg3);
        break;
    default:
        ret = -1;
        break;
    }
    if(!ret) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    return ret;
}

//Functions are not needed currently
#if 0
//Set the sensor output window
int set_output_window(sensor_t *sensor, uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    int ret = 0;
    uint16_t endx, endy;
    uint8_t com1, reg32;

    endy = y + height / 2;
    com1 = read_reg(sensor, BANK_SENSOR, COM1);
    WRITE_REG_OR_RETURN(BANK_SENSOR, COM1, (com1 & 0XF0) | (((endy & 0X03) << 2) | (y & 0X03)));
    WRITE_REG_OR_RETURN(BANK_SENSOR, VSTART, y >> 2);
    WRITE_REG_OR_RETURN(BANK_SENSOR, VSTOP, endy >> 2);

    endx = x + width / 2;
    reg32 = read_reg(sensor, BANK_SENSOR, REG32);
    WRITE_REG_OR_RETURN(BANK_SENSOR, REG32, (reg32 & 0XC0) | (((endx & 0X07) << 3) | (x & 0X07)));
    WRITE_REG_OR_RETURN(BANK_SENSOR, HSTART, x >> 3);
    WRITE_REG_OR_RETURN(BANK_SENSOR, HSTOP, endx >> 3);

    return ret;
}

// Set the image output size (final output resolution)
int set_output_size(sensor_t *sensor, uint16_t width, uint16_t height)
{
    int ret = 0;
    uint16_t h, w;

    if(width % 4) {
        return -1;
    }
    if(height % 4 ) {
        return -2;
    }

    w = width / 4;
    h = height / 4;
    //WRITE_REG_OR_RETURN(BANK_DSP, RESET, RESET_DVP);
    WRITE_REG_OR_RETURN(BANK_DSP, ZMOW, w & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, ZMOH, h & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, ZMHH, ((w >> 8) & 0X03) | ((h >> 6) & 0X04));
    //WRITE_REG_OR_RETURN(BANK_DSP, RESET, 0X00);

    return ret;
}

//Set the image window size >= output size
int set_window_size(sensor_t *sensor, uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    int ret = 0;
    uint16_t w, h;

    if(width % 4) {
        return -1;
    }
    if(height % 4) {
        return -2;
    }

    w = width / 4;
    h = height / 4;
    //WRITE_REG_OR_RETURN(BANK_DSP, RESET, RESET_DVP);
    WRITE_REG_OR_RETURN(BANK_DSP, HSIZE, w & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, VSIZE, h & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, XOFFL, x & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, YOFFL, y & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, VHYX, ((h >> 1) & 0X80) | ((y >> 4) & 0X70) | ((w >> 5) & 0X08) | ((x >> 8) & 0X07));
    WRITE_REG_OR_RETURN(BANK_DSP, TEST, (w >> 2) & 0X80);
    //WRITE_REG_OR_RETURN(BANK_DSP, RESET, 0X00);

    return ret;
}

//Set the sensor resolution (UXGA, SVGA, CIF)
int set_image_size(sensor_t *sensor, uint16_t width, uint16_t height)
{
    int ret = 0;
    //WRITE_REG_OR_RETURN(BANK_DSP, RESET, RESET_DVP);
    WRITE_REG_OR_RETURN(BANK_DSP, HSIZE8, (width >> 3) & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, VSIZE8, (height >> 3) & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, SIZEL, ((width & 0X07) << 3) | ((width >> 4) & 0X80) | (height & 0X07));
    //WRITE_REG_OR_RETURN(BANK_DSP, RESET, 0X00);

    return ret;
}
#endif

static int set_framesize(sensor_t *sensor, framesize_t framesize)
{
    int ret = 0;
    uint16_t w = resolution[framesize][0];
    uint16_t h = resolution[framesize][1];
    const uint8_t (*regs)[2];

    sensor->status.framesize = framesize;

    if (framesize <= FRAMESIZE_CIF) {
        regs = ov2640_settings_to_cif;
    } else if (framesize <= FRAMESIZE_SVGA) {
        regs = ov2640_settings_to_svga;
    } else {
        regs = ov2640_settings_to_uxga;
    }

    WRITE_REG_OR_RETURN(BANK_DSP, R_BYPASS, R_BYPASS_DSP_BYPAS);
    WRITE_REGS_OR_RETURN(regs);
    if (sensor->pixformat == PIXFORMAT_JPEG && sensor->xclk_freq_hz == 10000000) {
        if (framesize <= FRAMESIZE_CIF) {
            WRITE_REG_OR_RETURN(BANK_SENSOR, CLKRC, CLKRC_2X_CIF);
        } else if (framesize <= FRAMESIZE_SVGA) {
            WRITE_REG_OR_RETURN(BANK_SENSOR, CLKRC, CLKRC_2X_SVGA);
        } else {
            WRITE_REG_OR_RETURN(BANK_SENSOR, CLKRC, CLKRC_2X_UXGA);
        }
    }
    WRITE_REG_OR_RETURN(BANK_DSP, ZMOW, (w>>2)&0xFF); // OUTW[7:0] (real/4)
    WRITE_REG_OR_RETURN(BANK_DSP, ZMOH, (h>>2)&0xFF); // OUTH[7:0] (real/4)
    WRITE_REG_OR_RETURN(BANK_DSP, ZMHH, ((h>>8)&0x04)|((w>>10)&0x03)); // OUTH[8]/OUTW[9:8]
    WRITE_REG_OR_RETURN(BANK_DSP, RESET, 0x00);
    WRITE_REG_OR_RETURN(BANK_DSP, R_BYPASS, R_BYPASS_DSP_EN);

    vTaskDelay(10 / portTICK_PERIOD_MS);
    //required when changing resolution
    set_pixformat(sensor, sensor->pixformat);

    return ret;
}

static int set_contrast(sensor_t *sensor, int level)
{
    int ret=0;
    level += 3;
    if (level <= 0 || level > NUM_CONTRAST_LEVELS) {
        return -1;
    }
    sensor->status.contrast = level-3;
    for (int i=0; i<7; i++) {
        WRITE_REG_OR_RETURN(BANK_DSP, contrast_regs[0][i], contrast_regs[level][i]);
    }
    return ret;
}

static int set_brightness(sensor_t *sensor, int level)
{
    int ret=0;
    level += 3;
    if (level <= 0 || level > NUM_BRIGHTNESS_LEVELS) {
        return -1;
    }
    sensor->status.brightness = level-3;
    for (int i=0; i<5; i++) {
        WRITE_REG_OR_RETURN(BANK_DSP, brightness_regs[0][i], brightness_regs[level][i]);
    }
    return ret;
}

static int set_saturation(sensor_t *sensor, int level)
{
    int ret=0;
    level += 3;
    if (level <= 0 || level > NUM_SATURATION_LEVELS) {
        return -1;
    }
    sensor->status.saturation = level-3;
    for (int i=0; i<5; i++) {
        WRITE_REG_OR_RETURN(BANK_DSP, saturation_regs[0][i], saturation_regs[level][i]);
    }
    return ret;
}

static int set_special_effect(sensor_t *sensor, int effect)
{
    int ret=0;
    effect++;
    if (effect <= 0 || effect > NUM_SPECIAL_EFFECTS) {
        return -1;
    }
    sensor->status.special_effect = effect-1;
    for (int i=0; i<5; i++) {
        WRITE_REG_OR_RETURN(BANK_DSP, special_effects_regs[0][i], special_effects_regs[effect][i]);
    }
    return ret;
}

static int set_wb_mode(sensor_t *sensor, int mode)
{
    int ret=0;
    if (mode < 0 || mode > NUM_WB_MODES) {
        return -1;
    }
    sensor->status.wb_mode = mode;
    SET_REG_BITS_OR_RETURN(BANK_DSP, 0XC7, 6, 1, mode?1:0);
    if(mode) {
        for (int i=0; i<3; i++) {
            WRITE_REG_OR_RETURN(BANK_DSP, wb_modes_regs[0][i], wb_modes_regs[mode][i]);
        }
    }
    return ret;
}

static int set_ae_level(sensor_t *sensor, int level)
{
    int ret=0;
    level += 3;
    if (level <= 0 || level > NUM_AE_LEVELS) {
        return -1;
    }
    sensor->status.ae_level = level-3;
    for (int i=0; i<3; i++) {
        WRITE_REG_OR_RETURN(BANK_SENSOR, ae_levels_regs[0][i], ae_levels_regs[level][i]);
    }
    return ret;
}

static int set_quality(sensor_t *sensor, int quality)
{
    if(quality < 0) {
        quality = 0;
    } else if(quality > 63) {
        quality = 63;
    }
    sensor->status.quality = quality;
    return write_reg(sensor, BANK_DSP, QS, quality);
}

static int set_agc_gain(sensor_t *sensor, int gain)
{
    if(gain < 0) {
        gain = 0;
    } else if(gain > 30) {
        gain = 30;
    }
    sensor->status.agc_gain = gain;
    return write_reg(sensor, BANK_SENSOR, GAIN, agc_gain_tbl[gain]);
}

static int set_gainceiling_sensor(sensor_t *sensor, gainceiling_t gainceiling)
{
    sensor->status.gainceiling = gainceiling;
    //return write_reg(sensor, BANK_SENSOR, COM9, COM9_AGC_SET(gainceiling));
    return set_reg_bits(sensor, BANK_SENSOR, COM9, 5, 7, gainceiling);
}

static int set_aec_value(sensor_t *sensor, int value)
{
    if(value < 0) {
        value = 0;
    } else if(value > 1200) {
        value = 1200;
    }
    sensor->status.aec_value = value;
    return set_reg_bits(sensor, BANK_SENSOR, REG04, 0, 3, value & 0x3)
           || write_reg(sensor, BANK_SENSOR, AEC, (value >> 2) & 0xFF)
           || set_reg_bits(sensor, BANK_SENSOR, REG45, 0, 0x3F, value >> 10);
}

static int set_aec2(sensor_t *sensor, int enable)
{
    sensor->status.aec2 = enable;
    return set_reg_bits(sensor, BANK_DSP, CTRL0, 6, 1, enable?0:1);
}

static int set_colorbar(sensor_t *sensor, int enable)
{
    sensor->status.colorbar = enable;
    return write_reg_bits(sensor, BANK_SENSOR, COM7, COM7_COLOR_BAR, enable?1:0);
}

static int set_agc_sensor(sensor_t *sensor, int enable)
{
    sensor->status.agc = enable;
    return write_reg_bits(sensor, BANK_SENSOR, COM8, COM8_AGC_EN, enable?1:0);
}

static int set_aec_sensor(sensor_t *sensor, int enable)
{
    sensor->status.aec = enable;
    return write_reg_bits(sensor, BANK_SENSOR, COM8, COM8_AEC_EN, enable?1:0);
}

static int set_hmirror_sensor(sensor_t *sensor, int enable)
{
    sensor->status.hmirror = enable;
    return write_reg_bits(sensor, BANK_SENSOR, REG04, REG04_HFLIP_IMG, enable?1:0);
}

static int set_vflip_sensor(sensor_t *sensor, int enable)
{
    int ret = 0;
    sensor->status.vflip = enable;
    ret = write_reg_bits(sensor, BANK_SENSOR, REG04, REG04_VREF_EN, enable?1:0);
    return ret & write_reg_bits(sensor, BANK_SENSOR, REG04, REG04_VFLIP_IMG, enable?1:0);
}

static int set_raw_gma_dsp(sensor_t *sensor, int enable)
{
    sensor->status.raw_gma = enable;
    return set_reg_bits(sensor, BANK_DSP, CTRL1, 5, 1, enable?1:0);
}

static int set_awb_dsp(sensor_t *sensor, int enable)
{
    sensor->status.awb = enable;
    return set_reg_bits(sensor, BANK_DSP, CTRL1, 3, 1, enable?1:0);
}

static int set_awb_gain_dsp(sensor_t *sensor, int enable)
{
    sensor->status.awb_gain = enable;
    return set_reg_bits(sensor, BANK_DSP, CTRL1, 2, 1, enable?1:0);
}

static int set_lenc_dsp(sensor_t *sensor, int enable)
{
    sensor->status.lenc = enable;
    return set_reg_bits(sensor, BANK_DSP, CTRL1, 1, 1, enable?1:0);
}

static int set_dcw_dsp(sensor_t *sensor, int enable)
{
    sensor->status.dcw = enable;
    return set_reg_bits(sensor, BANK_DSP, CTRL2, 5, 1, enable?1:0);
}

static int set_bpc_dsp(sensor_t *sensor, int enable)
{
    sensor->status.bpc = enable;
    return set_reg_bits(sensor, BANK_DSP, CTRL3, 7, 1, enable?1:0);
}

static int set_wpc_dsp(sensor_t *sensor, int enable)
{
    sensor->status.wpc = enable;
    return set_reg_bits(sensor, BANK_DSP, CTRL3, 6, 1, enable?1:0);
}

//unsupported
static int set_sharpness(sensor_t *sensor, int level)
{
   return -1;
}

static int set_denoise(sensor_t *sensor, int level)
{
   return -1;
}

static int init_status(sensor_t *sensor){
    sensor->status.brightness = 0;
    sensor->status.contrast = 0;
    sensor->status.saturation = 0;
    sensor->status.ae_level = 0;
    sensor->status.special_effect = 0;
    sensor->status.wb_mode = 0;

    sensor->status.agc_gain = 30;
    int agc_gain = read_reg(sensor, BANK_SENSOR, GAIN);
    for (int i=0; i<30; i++){
        if(agc_gain >= agc_gain_tbl[i] && agc_gain < agc_gain_tbl[i+1]){
            sensor->status.agc_gain = i;
            break;
        }
    }

    sensor->status.aec_value = ((uint16_t)get_reg_bits(sensor, BANK_SENSOR, REG45, 0, 0x3F) << 10)
                             | ((uint16_t)read_reg(sensor, BANK_SENSOR, AEC) << 2)
                             | get_reg_bits(sensor, BANK_SENSOR, REG04, 0, 3);//0 - 1200
    sensor->status.quality = read_reg(sensor, BANK_DSP, QS);
    sensor->status.gainceiling = get_reg_bits(sensor, BANK_SENSOR, COM9, 5, 7);

    sensor->status.awb = get_reg_bits(sensor, BANK_DSP, CTRL1, 3, 1);
    sensor->status.awb_gain = get_reg_bits(sensor, BANK_DSP, CTRL1, 2, 1);
    sensor->status.aec = get_reg_bits(sensor, BANK_SENSOR, COM8, 0, 1);
    sensor->status.aec2 = get_reg_bits(sensor, BANK_DSP, CTRL0, 6, 1);
    sensor->status.agc = get_reg_bits(sensor, BANK_SENSOR, COM8, 2, 1);
    sensor->status.bpc = get_reg_bits(sensor, BANK_DSP, CTRL3, 7, 1);
    sensor->status.wpc = get_reg_bits(sensor, BANK_DSP, CTRL3, 6, 1);
    sensor->status.raw_gma = get_reg_bits(sensor, BANK_DSP, CTRL1, 5, 1);
    sensor->status.lenc = get_reg_bits(sensor, BANK_DSP, CTRL1, 1, 1);
    sensor->status.hmirror = get_reg_bits(sensor, BANK_SENSOR, REG04, 7, 1);
    sensor->status.vflip = get_reg_bits(sensor, BANK_SENSOR, REG04, 6, 1);
    sensor->status.dcw = get_reg_bits(sensor, BANK_DSP, CTRL2, 5, 1);
    sensor->status.colorbar = get_reg_bits(sensor, BANK_SENSOR, COM7, 1, 1);

    sensor->status.sharpness = 0;//not supported
    sensor->status.denoise = 0;
    return 0;
}

int ov2640_init(sensor_t *sensor)
{
    printf("attach 2640\n");
    sensor->reset = reset;
    sensor->init_status = init_status;
    sensor->set_pixformat = set_pixformat;
    sensor->set_framesize = set_framesize;
    sensor->set_contrast  = set_contrast;
    sensor->set_brightness= set_brightness;
    sensor->set_saturation= set_saturation;

    sensor->set_quality = set_quality;
    sensor->set_colorbar = set_colorbar;

    sensor->set_gainceiling = set_gainceiling_sensor;
    sensor->set_gain_ctrl = set_agc_sensor;
    sensor->set_exposure_ctrl = set_aec_sensor;
    sensor->set_hmirror = set_hmirror_sensor;
    sensor->set_vflip = set_vflip_sensor;

    sensor->set_whitebal = set_awb_dsp;
    sensor->set_aec2 = set_aec2;
    sensor->set_aec_value = set_aec_value;
    sensor->set_special_effect = set_special_effect;
    sensor->set_wb_mode = set_wb_mode;
    sensor->set_ae_level = set_ae_level;

    sensor->set_dcw = set_dcw_dsp;
    sensor->set_bpc = set_bpc_dsp;
    sensor->set_wpc = set_wpc_dsp;
    sensor->set_awb_gain = set_awb_gain_dsp;
    sensor->set_agc_gain = set_agc_gain;

    sensor->set_raw_gma = set_raw_gma_dsp;
    sensor->set_lenc = set_lenc_dsp;

    //not supported
    sensor->set_sharpness = set_sharpness;
    sensor->set_denoise = set_denoise;
    ESP_LOGD(TAG, "OV2640 Attached");
    return 0;
}

#if 0
///////////////////////////////////////////////

struct reg_raw {
  uint8_t reg;
  uint8_t cfg;
};


struct reg_raw uxga_cfg_tbl[] =
{
  {0xff, 0x00},
  {0x2c, 0xff},
  {0x2e, 0xdf},
  {0xff, 0x01},
  {0x3c, 0x32},
  // {0x11, 0x82},
  {0x11, 0x81},
  {0x09, 0x02},
  {0x04, 0x98},//水平镜像,垂直翻转
  {0x13, 0xe5},
  {0x14, 0x48},
  {0x2c, 0x0c},
  {0x33, 0x78},
  {0x3a, 0x33},
  {0x3b, 0xfB},
  {0x3e, 0x00},
  {0x43, 0x11},
  {0x16, 0x10},
  {0x39, 0x92},
  {0x35, 0xda},
  {0x22, 0x1a},
  {0x37, 0xc3},
  {0x23, 0x00},
  {0x34, 0xc0},
  {0x36, 0x1a},
  {0x06, 0x88},
  {0x07, 0xc0},
  {0x0d, 0x87},
  {0x0e, 0x41},
  {0x4c, 0x00},
  {0x48, 0x00},
  {0x5B, 0x00},
  {0x42, 0x03},
  {0x4a, 0x81},
  {0x21, 0x99},
  {0x24, 0x40},
  {0x25, 0x38},
  {0x26, 0x82},
  {0x5c, 0x00},
  {0x63, 0x00},
  {0x46, 0x00},
  {0x0c, 0x3c},
  {0x61, 0x70},
  {0x62, 0x80},
  {0x7c, 0x05},
  {0x20, 0x80},
  {0x28, 0x30},
  {0x6c, 0x00},
  {0x6d, 0x80},
  {0x6e, 0x00},
  {0x70, 0x02},
  {0x71, 0x94},
  {0x73, 0xc1},
  {0x3d, 0x34},
  {0x5a, 0x57},
  {0x12, 0x00},//UXGA 1600*1200
  {0x17, 0x11},
  {0x18, 0x75},
  {0x19, 0x01},
  {0x1a, 0x97},
  {0x32, 0x36},
  {0x03, 0x0f},
  {0x37, 0x40},
  {0x4f, 0xca},
  {0x50, 0xa8},
  {0x5a, 0x23},
  {0x6d, 0x00},
  {0x6d, 0x38},
  {0xff, 0x00},
  {0xe5, 0x7f},
  {0xf9, 0xc0},
  {0x41, 0x24},
  {0xe0, 0x14},
  {0x76, 0xff},
  {0x33, 0xa0},
  {0x42, 0x20},
  {0x43, 0x18},
  {0x4c, 0x00},
  {0x87, 0xd5},
  {0x88, 0x3f},
  {0xd7, 0x03},
  {0xd9, 0x10},
//    {0xd3, 0x82},
  {0xc8, 0x08},
  {0xc9, 0x80},
  {0x7c, 0x00},
  {0x7d, 0x00},
  {0x7c, 0x03},
  {0x7d, 0x48},
  {0x7d, 0x48},
  {0x7c, 0x08},
  {0x7d, 0x20},
  {0x7d, 0x10},
  {0x7d, 0x0e},
  {0x90, 0x00},
  {0x91, 0x0e},
  {0x91, 0x1a},
  {0x91, 0x31},
  {0x91, 0x5a},
  {0x91, 0x69},
  {0x91, 0x75},
  {0x91, 0x7e},
  {0x91, 0x88},
  {0x91, 0x8f},
  {0x91, 0x96},
  {0x91, 0xa3},
  {0x91, 0xaf},
  {0x91, 0xc4},
  {0x91, 0xd7},
  {0x91, 0xe8},
  {0x91, 0x20},
  {0x92, 0x00},
  {0x93, 0x06},
  {0x93, 0xe3},
  {0x93, 0x05},
  {0x93, 0x05},
  {0x93, 0x00},
  {0x93, 0x04},
  {0x93, 0x00},
  {0x93, 0x00},
  {0x93, 0x00},
  {0x93, 0x00},
  {0x93, 0x00},
  {0x93, 0x00},
  {0x93, 0x00},
  {0x96, 0x00},
  {0x97, 0x08},
  {0x97, 0x19},
  {0x97, 0x02},
  {0x97, 0x0c},
  {0x97, 0x24},
  {0x97, 0x30},
  {0x97, 0x28},
  {0x97, 0x26},
  {0x97, 0x02},
  {0x97, 0x98},
  {0x97, 0x80},
  {0x97, 0x00},
  {0x97, 0x00},
  {0xc3, 0xef},
  {0xa4, 0x00},
  {0xa8, 0x00},
  {0xc5, 0x11},
  {0xc6, 0x51},
  {0xbf, 0x80},
  {0xc7, 0x10},
  {0xb6, 0x66},
  {0xb8, 0xA5},
  {0xb7, 0x64},
  {0xb9, 0x7C},
  {0xb3, 0xaf},
  {0xb4, 0x97},
  {0xb5, 0xFF},
  {0xb0, 0xC5},
  {0xb1, 0x94},
  {0xb2, 0x0f},
  {0xc4, 0x5c},
  {0xc0, 0xc8},
  {0xc1, 0x96},
  {0x8c, 0x00},
  {0x86, 0x3d},
  {0x50, 0x00},
  {0x51, 0x90},
  {0x52, 0x2c},
  {0x53, 0x00},
  {0x54, 0x00},
  {0x55, 0x88},
  {0x5a, 0x90},
  {0x5b, 0x2C},
  {0x5c, 0x05},
  {0xd3, 0x04},//auto设置要小心
  {0xc3, 0xed},
  {0x7f, 0x00},
  {0xda, 0x09},
  {0xe5, 0x1f},
  {0xe1, 0x67},
  {0xe0, 0x00},
  {0xdd, 0x7f},
  {0x05, 0x00},
};

struct reg_raw rgb565_cfg_table[] = {
  {0xFF, 0x00},
  {0xe0, 0x04},
  {0xDA, 0x08},

  {0xD7, 0x01},
  {0xe1, 0x77},
  {0xe0, 0x00},
};

void ov2640_recfg(void)
{
    return;
  printf("recfg OV\n");
  uint8_t id = 0x0;
  sccb_write_reg(0xFF, 0x01);//bank sensor
  sccb_write_reg(0x12, 0x80);//reset

  uint8_t pid = 0, ver = 0, midl = 0, midh = 0;

  if (sccb_read_reg(0x0A, &pid) != 0) {
    printf("read fail\n");
  }
  if (sccb_read_reg(0x0B, &ver) != 0) {
    printf("read fail\n");
  }
  if (sccb_read_reg(0x1C, &midh) != 0) {
    printf("read fail\n");
  }
  if (sccb_read_reg(0x1D, &midl) != 0) {
    printf("read fail\n");
  }
  printf("0x%x  0x%x  0x%x  0x%x\n", pid, ver, midl, midh);
  for (int i = 0; i < sizeof(uxga_cfg_tbl) / sizeof(struct reg_raw); i++) {
    sccb_write_reg(uxga_cfg_tbl[i].reg, uxga_cfg_tbl[i].cfg);
  }
  ets_delay_us(10000);
  for (int i = 0; i < sizeof(rgb565_cfg_table) / sizeof(struct reg_raw); i++) {
    sccb_write_reg(rgb565_cfg_table[i].reg, rgb565_cfg_table[i].cfg);
  }

  uint16_t hsize;
  uint16_t vsize;
  uint8_t temp;
  hsize = 1600 / 4;
  vsize = 1200 / 4;
  sccb_write_reg(0XFF, 0X00);
  sccb_write_reg(0XE0, 0X04);
  sccb_write_reg(0X51, hsize & 0XFF); //设置H_SIZE的低八位
  sccb_write_reg(0X52, vsize & 0XFF); //设置V_SIZE的低八位
  sccb_write_reg(0X53, 0 & 0XFF);   //设置offx的低八位
  sccb_write_reg(0X54, 0 & 0XFF);   //设置offy的低八位
  temp = (vsize >> 1) & 0X80;
  temp |= (0 >> 4) & 0X70;
  temp |= (hsize >> 5) & 0X08;
  temp |= (0 >> 8) & 0X07;
  sccb_write_reg(0X55, temp);      //设置H_SIZE/V_SIZE/OFFX,OFFY的高位
  sccb_write_reg(0X57, (hsize >> 2) & 0X80); //设置H_SIZE/V_SIZE/OFFX,OFFY的高位
  sccb_write_reg(0XE0, 0X00);

  uint8_t outw = 320 / 4;
  uint8_t outh = 240 / 4;
  sccb_write_reg(0XFF, 0X00);
  sccb_write_reg(0XE0, 0X04);
  sccb_write_reg(0X5A, outw & 0XFF); //设置OUTW的低八位
  sccb_write_reg(0X5B, outh & 0XFF); //设置OUTH的低八位
  temp = (outw >> 8) & 0X03;
  temp |= (outh >> 6) & 0X04;
  sccb_write_reg(0X5C, temp);      //设置OUTH/OUTW的高位
  sccb_write_reg(0XE0, 0X00);
  sccb_write_reg(0XFF, 0X00);

  /*  VYUY   */
  sccb_read_reg(0xda, &temp);
  temp |= 0x01;
  sccb_write_reg(0xda, temp);
#if 0
  sccb_read_reg(0xc2, &temp);
  temp &= 0xef;
  sccb_write_reg(0xc2, temp);

  sccb_write_reg(0x00, 0X00);
#endif
}

//////////////////////////////

#endif