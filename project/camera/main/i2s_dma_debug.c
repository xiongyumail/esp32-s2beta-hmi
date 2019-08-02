#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_system.h"
#include <math.h>
#include "soc/i2s_struct.h"
#include"soc/apb_ctrl_reg.h"

#include "soc/i2s_struct.h"
#include "esp32s2beta/rom/lldesc.h"
#include "soc/dport_access.h"

#define DPORT_I2S0_CLK_EN   (BIT(4))
#define DPORT_I2S0_RST   (BIT(4))

#define DPORT_PERIP_CLK_EN0_REG          (DR_REG_SYSTEM_BASE + 0x040)
#define DPORT_PERIP_CLK_EN_REG DPORT_PERIP_CLK_EN0_REG
#define DPORT_PERIP_RST_EN0_REG          (DR_REG_SYSTEM_BASE + 0x048)
#define DPORT_PERIP_RST_EN_REG DPORT_PERIP_RST_EN0_REG

static void i2s_config(void)
{
    REG_WRITE(APB_CTRL_SARADC_CTRL_REG, REG_READ(APB_CTRL_SARADC_CTRL_REG) & (~APB_CTRL_SARADC_DATA_TO_I2S));
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_I2S0_CLK_EN);
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_I2S0_CLK_EN);
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2S0_RST);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2S0_RST);
    I2S0.clkm_conf.val = 0;
    I2S0.clkm_conf.clkm_div_num = 4; //recv 8 32 :4
    I2S0.clkm_conf.clkm_div_b = 6;   //6
    I2S0.clkm_conf.clkm_div_a =2;  //2
    I2S0.clkm_conf.clk_en = 1;
    I2S0.clkm_conf.clk_sel = 2;

    I2S0.int_ena.val = 0;
    I2S0.int_clr.val = ~0;
    I2S0.conf.val = 0;
    I2S0.conf1.val = 0;
    I2S0.conf1.tx_pcm_bypass = 1;
    I2S0.conf1.rx_pcm_bypass = 1;
    I2S0.conf1.tx_stop_en = 1;
    I2S0.sample_rate_conf.val = 0;
    I2S0.sample_rate_conf.tx_bck_div_num = 12;
    I2S0.sample_rate_conf.rx_bck_div_num = 12;
    I2S0.timing.val = 0;
    I2S0.conf2.val = 0;
    I2S0.fifo_conf.val = 0;
    I2S0.fifo_conf.tx_fifo_mod_force_en = 1;
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    I2S0.conf_single_data = 0x9abcdef0;
    I2S0.fifo_conf.tx_data_num = 32;
    I2S0.fifo_conf.rx_data_num = 32;
    printf("i2s version  0x%x\n", I2S0.date);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[18], PIN_FUNC_GPIO);
    gpio_set_direction(18, GPIO_MODE_DEF_OUTPUT);
    gpio_matrix_out(18, I2S0I_BCK_OUT_IDX, false, 0);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[19], PIN_FUNC_GPIO);
    gpio_set_direction(19, GPIO_MODE_DEF_OUTPUT);
    gpio_matrix_out(19, I2S0I_WS_OUT_IDX, false, 0);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[5], PIN_FUNC_GPIO);
    gpio_set_direction(5, GPIO_MODE_DEF_INPUT);
    gpio_matrix_in(5, I2S0I_DATA_IN15_IDX, 0);

    I2S0.sample_rate_conf.rx_bits_mod = 8;
    I2S0.conf_chan.rx_chan_mod = 0;
    I2S0.conf.rx_slave_mod = 0;
    I2S0.conf.rx_dma_equal = 0;
    // I2S0.lc_conf.indscr_burst_en = 1;
}

static lldesc_t _dma = {0};

static void i2s_recv_dma_prog(void)
{
    const int RX_EOF_NUM = 180;
    const int  BUF_SIZE = 180;
    I2S0.lc_conf.val = 0;
    I2S0.lc_conf.in_rst  = 1;
    I2S0.lc_conf.in_rst  = 0;
    I2S0.lc_conf.out_rst  = 1;
    I2S0.lc_conf.out_rst  = 0;
    I2S0.lc_conf.ahbm_fifo_rst = 1;
    I2S0.lc_conf.ahbm_fifo_rst = 0;
    I2S0.lc_conf.ahbm_rst = 1;
    I2S0.lc_conf.ahbm_rst = 0;
    I2S0.lc_conf.out_eof_mode = 1;
    I2S0.lc_conf.out_data_burst_en = 1;
    I2S0.lc_conf.outdscr_burst_en = 1;
    I2S0.lc_conf.indscr_burst_en = 1;
    I2S0.lc_conf.check_owner = 0;
    I2S0.fifo_conf.dscr_en = 1;
    I2S0.out_link.val = 0;
    I2S0.in_link.val = 0;
    uint32_t *buf = (uint32_t*)calloc(1, sizeof(uint32_t)* BUF_SIZE + 7);
    for(int i = 0; i < 100; i++) {
        buf[i] = 0;;
    }
    _dma.offset = 0;
    _dma.size = sizeof(uint32_t)* BUF_SIZE;
    _dma.length = sizeof(uint32_t)* BUF_SIZE;
    _dma.sosf = 0;
    _dma.eof = 1;
    _dma.owner = 1;
    _dma.buf = (uint8_t*)buf;
    _dma.empty = NULL;
    I2S0.rx_eof_num = sizeof(uint32_t)* RX_EOF_NUM;
    I2S0.int_clr.val = ~0;
    I2S0.in_link.addr = ((uint32_t)&_dma) & 0xfffff;
    I2S0.int_ena.in_suc_eof = 1;
    I2S0.fifo_conf.dscr_en = 1;
    I2S0.in_link.start = 1;
    I2S0.conf.rx_start = 1;
    while(!I2S0.int_raw.in_suc_eof);
    for(int i = 0; i < 100; i++) {
        printf("%x\n", buf[i]);
    }
}

void i2s_dma_debug(void)
{
    i2s_config();
    i2s_recv_dma_prog();
}