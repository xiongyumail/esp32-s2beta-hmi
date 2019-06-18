#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_system.h"
#include <math.h>
#include "soc/i2s_struct.h"
#include "soc/apb_ctrl_reg.h"
#include "rom/lldesc.h"

/*   I2S LCD interface configure part  */

#define  WR  GPIO_NUM_4
#define  RS  GPIO_NUM_3

#define  D0  GPIO_NUM_20
#define  D1  GPIO_NUM_19
#define  D2  GPIO_NUM_18
#define  D3  GPIO_NUM_17
#define  D4  GPIO_NUM_16
#define  D5  GPIO_NUM_15
#define  D6  GPIO_NUM_14
#define  D7  GPIO_NUM_13

#define  D8  GPIO_NUM_12
#define  D9  GPIO_NUM_11
#define  D10  GPIO_NUM_10
#define  D11  GPIO_NUM_9
#define  D12  GPIO_NUM_8
#define  D13  GPIO_NUM_7
#define  D14  GPIO_NUM_6
#define  D15  GPIO_NUM_5

#define RS_LOW()   gpio_set_level(RS, 0)
#define RS_HIGH()  gpio_set_level(RS, 1)

#define LCD_DELAY_50US(n)   ets_delay_us(50 * n)

typedef struct {
    uint8_t bit_width;
    uint8_t pin_clk;
    uint8_t pin_rs;
    uint8_t data[16];
} lcd_pin_def_t;


lcd_pin_def_t lcd_bus = {
    .bit_width = 16,
    .pin_clk = WR,
    .pin_rs = RS,
    .data = {D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, D14, D15},
};

static lldesc_t __dma[250] = {0};

int raw = 320, col = 480;

void i2s_lcd_config(void)
{
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_I2S0_CLK_EN);
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_I2S0_CLK_EN);
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2S0_RST);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2S0_RST);

    //Configure pclk, max: 10M
    I2S0.clkm_conf.val = 0;
    I2S0.clkm_conf.clkm_div_num = 2;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_a =63;
    I2S0.clkm_conf.clk_en = 1;
    I2S0.clkm_conf.clk_sel = 2;
    I2S0.sample_rate_conf.val = 0;
    I2S0.sample_rate_conf.tx_bck_div_num = 2;

    I2S0.int_ena.val = 0;
    I2S0.int_clr.val = ~0;

    I2S0.conf.val = 0;
    I2S0.conf.tx_right_first = 1;
    I2S0.conf.tx_msb_right = 1;
    I2S0.conf.tx_dma_equal = 1;

    I2S0.conf1.val = 0;
    I2S0.conf1.tx_pcm_bypass = 1;
    I2S0.conf1.tx_stop_en = 1;
    I2S0.timing.val = 0;
    //Set LCD mode
    I2S0.conf2.val = 0;
    I2S0.conf2.lcd_en = 1;

    I2S0.fifo_conf.val = 0;
    I2S0.fifo_conf.tx_fifo_mod_force_en = 1;
    I2S0.fifo_conf.tx_data_num = 32;
    I2S0.fifo_conf.tx_fifo_mod = 3;

    I2S0.conf_chan.tx_chan_mod = 0;//remove

    I2S0.sample_rate_conf.tx_bits_mod = 16;
    printf("--------I2S version  %x\n", I2S0.date);
}

void i2s_lcd_set_pin(void)
{
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[21], PIN_FUNC_GPIO);
    gpio_set_direction(21, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(21,GPIO_PULLUP_ONLY);
    gpio_matrix_out(21, CLK_I2S_MUX_IDX, true, 0);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[lcd_bus.pin_clk], PIN_FUNC_GPIO);
    gpio_set_direction(lcd_bus.pin_clk, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(lcd_bus.pin_clk,GPIO_PULLUP_ONLY);
    gpio_matrix_out(lcd_bus.pin_clk, I2S0O_WS_OUT_IDX, true, 0);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[lcd_bus.pin_rs], PIN_FUNC_GPIO);
    gpio_set_direction(lcd_bus.pin_rs, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(lcd_bus.pin_rs,GPIO_PULLUP_ONLY);

    for(int i = 0; i < lcd_bus.bit_width; i++) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[lcd_bus.data[i]], PIN_FUNC_GPIO);
        gpio_set_direction(lcd_bus.data[i], GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(lcd_bus.data[i],GPIO_PULLUP_ONLY);
        gpio_matrix_out(lcd_bus.data[i], I2S0O_DATA_OUT8_IDX+i, false, 0);
    }
}

typedef struct i2s_isr_obj {
    uint8_t *fram_end;
    uint8_t *fram_sart;
    uint8_t *fram_cur;
    uint16_t tangel;
} i2s_isr_obj_t;

static i2s_isr_obj_t i2s_fram_obj = {0};
static int fram_ena_flag = 0;

void i2s_lcd_if_init(void)
{
    for(int i = 0; i < 30; i++) {
        __dma[i].size = 0;
        __dma[i].length = 0;
        __dma[i].sosf = 0;
        __dma[i].eof = 1;
        __dma[i].owner = 1;
        __dma[i].buf = NULL;
        __dma[i].empty = NULL;
    }
    i2s_lcd_set_pin();
    i2s_lcd_config();
}

static void _i2s_isr(void * param)
{
//    ets_printf("r\n");
    i2s_isr_obj_t *p = (i2s_isr_obj_t *)param;
    
    if(I2S0.int_st.out_eof) {
        lldesc_t *head = &__dma[0];
        if(p->tangel == 0) {
            head = &__dma[10];
        }
        for(int i = 0; i < 10; i++) {
            head->owner = 1;
            head->buf = p->fram_cur;
            p->fram_cur += 3200;
            head++;
        }
        p->tangel ^= 0x1;
        if(p->fram_cur == p->fram_end) {
            p->fram_cur = p->fram_sart;
        }
    }
    I2S0.int_clr.val = I2S0.int_st.val;
}

static inline void i2s_dma_start(void)
{
    I2S0.out_link.start = 1;
    I2S0.fifo_conf.dscr_en = 1;
    I2S0.conf.tx_start = 1;
    while(!I2S0.int_raw.out_eof);
    while (!(I2S0.state.tx_idle));
    I2S0.int_clr.val = ~0;
    I2S0.conf.tx_start = 0;
    I2S0.fifo_conf.dscr_en = 0;
    I2S0.conf.tx_reset = 1;
    I2S0.conf.tx_reset = 0;
}

void i2s_lcd_dma_write(uint8_t *buf, size_t length)
{
    __dma[0].size = length;
    __dma[0].length = length;
    __dma[0].buf = buf;
    __dma[0].eof = 1;
    I2S0.out_link.addr = ((uint32_t)&__dma[0]) & 0xfffff;
    i2s_dma_start();
}
#include "ppc.h"
#include "rom/cache.h"

#define DESC__NUM (20)

void lcd_draw_bmp(uint8_t *pic);

void i2s_lcd_ena_fram_buf(void)
{
    uint8_t *pfram = (uint8_t *)heap_caps_calloc(1, sizeof(uint16_t) * 800 * 480, MALLOC_CAP_SPIRAM);
    if(pfram == NULL){
        printf("fram buf malloc fail\n");
        return;
    } else {
        printf("%p\n", pfram);
    }
    uint8_t *ptr = (uint8_t *)pfram;
    for(int i = 0; i < DESC__NUM; i++) {
        __dma[i].size = 3200;
        __dma[i].length = 3200;
        __dma[i].owner = 1;
        __dma[i].buf = ptr;
        __dma[i].eof = 0;
        __dma[i].empty = &__dma[(i+1) % DESC__NUM];
        ptr += 3200;
        if(i == 9 || i == 19) {
            __dma[i].eof = 1;
        }
    }
    i2s_fram_obj.fram_end = pfram + 800 *480 * 2;
    i2s_fram_obj.fram_sart = pfram;
    i2s_fram_obj.fram_cur = ptr;
    i2s_fram_obj.tangel = 0;
    I2S0.int_ena.val = 0;
    I2S0.int_clr.val = ~0x0;
    uint16_t *p2 = (uint16_t *)pfram;
    uint16_t *p1 = (uint16_t *)gImage_pic;
    for(int i = 0; i < 800 * 480; i++) {
        p2[i] = p1[i];
    }
    Cache_WriteBack_All();
    lcd_draw_bmp(pfram);
    // vTaskDelay(3000/portTICK_PERIOD_MS);
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE, 0, _i2s_isr, (void *)&i2s_fram_obj, NULL);
    I2S0.int_ena.out_eof = 1;
    fram_ena_flag = 1;
    I2S0.out_link.addr = ((uint32_t)&__dma[0]) & 0xfffff;
    I2S0.fifo_conf.dscr_en = 1;
    I2S0.out_link.start = 1;
    I2S0.conf.tx_start = 1;
    // i2s_lcd_dma_write(pfram, 800*2 * 2);
}

void lcd_write_reg(uint16_t cmd, uint16_t data)
{
#if 1
    RS_LOW();
    i2s_lcd_dma_write(&cmd, 2);
    RS_HIGH();
    i2s_lcd_dma_write(&data, 2);
#else
    RS_LOW();
    // i2s_lcd_dma_write(&cmd, 2);
    REG_WRITE(0x6000f000, cmd<<16);
    I2S0.conf.tx_start = 1;
    while (!(I2S0.state.tx_idle));
    I2S0.conf.tx_start = 0;
    I2S0.conf.tx_reset = 1;
    I2S0.conf.tx_reset = 0;
    RS_HIGH();
    // i2s_lcd_dma_write(&data, 2);
    REG_WRITE(0x6000f000, data<<16);
    I2S0.conf.tx_start = 1;
    while (!(I2S0.state.tx_idle));
    I2S0.conf.tx_start = 0;
    I2S0.conf.tx_reset = 1;
    I2S0.conf.tx_reset = 0;
#endif
}

void lcd_write_cmd_byte(uint16_t cmd)
{
    RS_LOW();
#if 1
    i2s_lcd_dma_write(&cmd, 2);
#else
    REG_WRITE(0x6000f000, cmd<<16);
    I2S0.conf.tx_start = 1;
    while (!(I2S0.state.tx_idle));
    I2S0.conf.tx_start = 0;
    I2S0.conf.tx_reset = 1;
    I2S0.conf.tx_reset = 0;
#endif
    RS_HIGH();
}

void lcd_write_data_byte(uint16_t data)
{
#if 1
    i2s_lcd_dma_write(&data, 2);
#else
    REG_WRITE(0x6000f000, data<<16);
    I2S0.conf.tx_start = 1;
    while (!(I2S0.state.tx_idle));
    I2S0.conf.tx_start = 0;
    I2S0.conf.tx_reset = 1;
    I2S0.conf.tx_reset = 0;
#endif
}

void lcd_write_data_brust(uint8_t *buf, size_t length)
{
    i2s_lcd_dma_write(buf, length);
}

void lcd_set_win(uint16_t x_start, uint16_t x_end, uint16_t y_start, uint16_t y_end)
{
    lcd_write_reg(0x2A00, (x_start >> 8));
    lcd_write_reg(0x2A01, (x_start & 0xff));
    lcd_write_reg(0x2A02, (x_end >> 8));
    lcd_write_reg(0x2A03, (x_end & 0xff));

    lcd_write_reg(0x2B00, (y_start >> 8));
    lcd_write_reg(0x2B01, (y_start & 0xff));
    lcd_write_reg(0x2B02, (y_end >> 8));
    lcd_write_reg(0x2B03, (y_end & 0xff));

    lcd_write_cmd_byte(0x2C00);
}

void nt35510_init(void)
{
    i2s_lcd_if_init();
    LCD_DELAY_50US(10);
    lcd_write_cmd_byte(0x0100);
    lcd_write_cmd_byte(0x0100);
    LCD_DELAY_50US(100);
    lcd_write_cmd_byte(0x1200);
    lcd_write_reg(0xf000, 0x0055);
    lcd_write_reg(0xf001, 0x00aa);
    lcd_write_reg(0xf002, 0x0052);
    lcd_write_reg(0xf003, 0x0008);
    lcd_write_reg(0xf004, 0x0001);

    lcd_write_reg(0xbc01, 0x0086);
    lcd_write_reg(0xbc02, 0x006a);
    lcd_write_reg(0xbd01, 0x0086);
    lcd_write_reg(0xbd02, 0x006a);
    lcd_write_reg(0xbe01, 0x0067);

    lcd_write_reg(0xd100, 0x0000);
    lcd_write_reg(0xd101, 0x005d);
    lcd_write_reg(0xd102, 0x0000);
    lcd_write_reg(0xd103, 0x006b);
    lcd_write_reg(0xd104, 0x0000);
    lcd_write_reg(0xd105, 0x0084);
    lcd_write_reg(0xd106, 0x0000);
    lcd_write_reg(0xd107, 0x009c);
    lcd_write_reg(0xd108, 0x0000);
    lcd_write_reg(0xd109, 0x00b1);
    lcd_write_reg(0xd10a, 0x0000);
    lcd_write_reg(0xd10b, 0x00d9);
    lcd_write_reg(0xd10c, 0x0000);
    lcd_write_reg(0xd10d, 0x00fd);
    lcd_write_reg(0xd10e, 0x0001);
    lcd_write_reg(0xd10f, 0x0038);
    lcd_write_reg(0xd110, 0x0001);
    lcd_write_reg(0xd111, 0x0068);
    lcd_write_reg(0xd112, 0x0001);
    lcd_write_reg(0xd113, 0x00b9);
    lcd_write_reg(0xd114, 0x0001);
    lcd_write_reg(0xd115, 0x00fb);
    lcd_write_reg(0xd116, 0x0002);
    lcd_write_reg(0xd117, 0x0063);
    lcd_write_reg(0xd118, 0x0002);
    lcd_write_reg(0xd119, 0x00b9);
    lcd_write_reg(0xd11a, 0x0002);
    lcd_write_reg(0xd11b, 0x00bb);
    lcd_write_reg(0xd11c, 0x0003);
    lcd_write_reg(0xd11d, 0x0003);
    lcd_write_reg(0xd11e, 0x0003);
    lcd_write_reg(0xd11f, 0x0046);
    lcd_write_reg(0xd120, 0x0003);
    lcd_write_reg(0xd121, 0x0069);
    lcd_write_reg(0xd122, 0x0003);
    lcd_write_reg(0xd123, 0x008f);
    lcd_write_reg(0xd124, 0x0003);
    lcd_write_reg(0xd125, 0x00a4);
    lcd_write_reg(0xd126, 0x0003);
    lcd_write_reg(0xd127, 0x00b9);
    lcd_write_reg(0xd128, 0x0003);
    lcd_write_reg(0xd129, 0x00c7);
    lcd_write_reg(0xd12a, 0x0003);
    lcd_write_reg(0xd12b, 0x00c9);
    lcd_write_reg(0xd12c, 0x0003);
    lcd_write_reg(0xd12d, 0x00cb);
    lcd_write_reg(0xd12e, 0x0003);
    lcd_write_reg(0xd12f, 0x00cb);
    lcd_write_reg(0xd130, 0x0003);
    lcd_write_reg(0xd131, 0x00cb);
    lcd_write_reg(0xd132, 0x0003);
    lcd_write_reg(0xd133, 0x00cc);

    lcd_write_reg(0xd200, 0x0000);
    lcd_write_reg(0xd201, 0x005d);
    lcd_write_reg(0xd202, 0x0000);
    lcd_write_reg(0xd203, 0x006b);
    lcd_write_reg(0xd204, 0x0000);
    lcd_write_reg(0xd205, 0x0084);
    lcd_write_reg(0xd206, 0x0000);
    lcd_write_reg(0xd207, 0x009c);
    lcd_write_reg(0xd208, 0x0000);
    lcd_write_reg(0xd209, 0x00b1);
    lcd_write_reg(0xd20a, 0x0000);
    lcd_write_reg(0xd20b, 0x00d9);
    lcd_write_reg(0xd20c, 0x0000);
    lcd_write_reg(0xd20d, 0x00fd);
    lcd_write_reg(0xd20e, 0x0001);
    lcd_write_reg(0xd20f, 0x0038);
    lcd_write_reg(0xd210, 0x0001);
    lcd_write_reg(0xd211, 0x0068);
    lcd_write_reg(0xd212, 0x0001);
    lcd_write_reg(0xd213, 0x00b9);
    lcd_write_reg(0xd214, 0x0001);
    lcd_write_reg(0xd215, 0x00fb);
    lcd_write_reg(0xd216, 0x0002);
    lcd_write_reg(0xd217, 0x0063);
    lcd_write_reg(0xd218, 0x0002);
    lcd_write_reg(0xd219, 0x00b9);
    lcd_write_reg(0xd21a, 0x0002);
    lcd_write_reg(0xd21b, 0x00bb);
    lcd_write_reg(0xd21c, 0x0003);
    lcd_write_reg(0xd21d, 0x0003);
    lcd_write_reg(0xd21e, 0x0003);
    lcd_write_reg(0xd21f, 0x0046);
    lcd_write_reg(0xd220, 0x0003);
    lcd_write_reg(0xd221, 0x0069);
    lcd_write_reg(0xd222, 0x0003);
    lcd_write_reg(0xd223, 0x008f);
    lcd_write_reg(0xd224, 0x0003);
    lcd_write_reg(0xd225, 0x00a4);
    lcd_write_reg(0xd226, 0x0003);
    lcd_write_reg(0xd227, 0x00b9);
    lcd_write_reg(0xd228, 0x0003);
    lcd_write_reg(0xd229, 0x00c7);
    lcd_write_reg(0xd22a, 0x0003);
    lcd_write_reg(0xd22b, 0x00c9);
    lcd_write_reg(0xd22c, 0x0003);
    lcd_write_reg(0xd22d, 0x00cb);
    lcd_write_reg(0xd22e, 0x0003);
    lcd_write_reg(0xd22f, 0x00cb);
    lcd_write_reg(0xd230, 0x0003);
    lcd_write_reg(0xd231, 0x00cb);
    lcd_write_reg(0xd232, 0x0003);
    lcd_write_reg(0xd233, 0x00cc);

    lcd_write_reg(0xd300, 0x0000);
    lcd_write_reg(0xd301, 0x005d);
    lcd_write_reg(0xd302, 0x0000);
    lcd_write_reg(0xd303, 0x006b);
    lcd_write_reg(0xd304, 0x0000);
    lcd_write_reg(0xd305, 0x0084);
    lcd_write_reg(0xd306, 0x0000);
    lcd_write_reg(0xd307, 0x009c);
    lcd_write_reg(0xd308, 0x0000);
    lcd_write_reg(0xd309, 0x00b1);
    lcd_write_reg(0xd30a, 0x0000);
    lcd_write_reg(0xd30b, 0x00d9);
    lcd_write_reg(0xd30c, 0x0000);
    lcd_write_reg(0xd30d, 0x00fd);
    lcd_write_reg(0xd30e, 0x0001);
    lcd_write_reg(0xd30f, 0x0038);
    lcd_write_reg(0xd310, 0x0001);
    lcd_write_reg(0xd311, 0x0068);
    lcd_write_reg(0xd312, 0x0001);
    lcd_write_reg(0xd313, 0x00b9);
    lcd_write_reg(0xd314, 0x0001);
    lcd_write_reg(0xd315, 0x00fb);
    lcd_write_reg(0xd316, 0x0002);
    lcd_write_reg(0xd317, 0x0063);
    lcd_write_reg(0xd318, 0x0002);
    lcd_write_reg(0xd319, 0x00b9);
    lcd_write_reg(0xd31a, 0x0002);
    lcd_write_reg(0xd31b, 0x00bb);
    lcd_write_reg(0xd31c, 0x0003);
    lcd_write_reg(0xd31d, 0x0003);
    lcd_write_reg(0xd31e, 0x0003);
    lcd_write_reg(0xd31f, 0x0046);
    lcd_write_reg(0xd320, 0x0003);
    lcd_write_reg(0xd321, 0x0069);
    lcd_write_reg(0xd322, 0x0003);
    lcd_write_reg(0xd323, 0x008f);
    lcd_write_reg(0xd324, 0x0003);
    lcd_write_reg(0xd325, 0x00a4);
    lcd_write_reg(0xd326, 0x0003);
    lcd_write_reg(0xd327, 0x00b9);
    lcd_write_reg(0xd328, 0x0003);
    lcd_write_reg(0xd329, 0x00c7);
    lcd_write_reg(0xd32a, 0x0003);
    lcd_write_reg(0xd32b, 0x00c9);
    lcd_write_reg(0xd32c, 0x0003);
    lcd_write_reg(0xd32d, 0x00cb);
    lcd_write_reg(0xd32e, 0x0003);
    lcd_write_reg(0xd32f, 0x00cb);
    lcd_write_reg(0xd330, 0x0003);
    lcd_write_reg(0xd331, 0x00cb);
    lcd_write_reg(0xd332, 0x0003);
    lcd_write_reg(0xd333, 0x00cc);

    lcd_write_reg(0xd400, 0x0000);
    lcd_write_reg(0xd401, 0x005d);
    lcd_write_reg(0xd402, 0x0000);
    lcd_write_reg(0xd403, 0x006b);
    lcd_write_reg(0xd404, 0x0000);
    lcd_write_reg(0xd405, 0x0084);
    lcd_write_reg(0xd406, 0x0000);
    lcd_write_reg(0xd407, 0x009c);
    lcd_write_reg(0xd408, 0x0000);
    lcd_write_reg(0xd409, 0x00b1);
    lcd_write_reg(0xd40a, 0x0000);
    lcd_write_reg(0xd40b, 0x00d9);
    lcd_write_reg(0xd40c, 0x0000);
    lcd_write_reg(0xd40d, 0x00fd);
    lcd_write_reg(0xd40e, 0x0001);
    lcd_write_reg(0xd40f, 0x0038);
    lcd_write_reg(0xd410, 0x0001);
    lcd_write_reg(0xd411, 0x0068);
    lcd_write_reg(0xd412, 0x0001);
    lcd_write_reg(0xd413, 0x00b9);
    lcd_write_reg(0xd414, 0x0001);
    lcd_write_reg(0xd415, 0x00fb);
    lcd_write_reg(0xd416, 0x0002);
    lcd_write_reg(0xd417, 0x0063);
    lcd_write_reg(0xd418, 0x0002);
    lcd_write_reg(0xd419, 0x00b9);
    lcd_write_reg(0xd41a, 0x0002);
    lcd_write_reg(0xd41b, 0x00bb);
    lcd_write_reg(0xd41c, 0x0003);
    lcd_write_reg(0xd41d, 0x0003);
    lcd_write_reg(0xd41e, 0x0003);
    lcd_write_reg(0xd41f, 0x0046);
    lcd_write_reg(0xd420, 0x0003);
    lcd_write_reg(0xd421, 0x0069);
    lcd_write_reg(0xd422, 0x0003);
    lcd_write_reg(0xd423, 0x008f);
    lcd_write_reg(0xd424, 0x0003);
    lcd_write_reg(0xd425, 0x00a4);
    lcd_write_reg(0xd426, 0x0003);
    lcd_write_reg(0xd427, 0x00b9);
    lcd_write_reg(0xd428, 0x0003);
    lcd_write_reg(0xd429, 0x00c7);
    lcd_write_reg(0xd42a, 0x0003);
    lcd_write_reg(0xd42b, 0x00c9);
    lcd_write_reg(0xd42c, 0x0003);
    lcd_write_reg(0xd42d, 0x00cb);
    lcd_write_reg(0xd42e, 0x0003);
    lcd_write_reg(0xd42f, 0x00cb);
    lcd_write_reg(0xd430, 0x0003);
    lcd_write_reg(0xd431, 0x00cb);
    lcd_write_reg(0xd432, 0x0003);
    lcd_write_reg(0xd433, 0x00cc);

    lcd_write_reg(0xd500, 0x0000);
    lcd_write_reg(0xd501, 0x005d);
    lcd_write_reg(0xd502, 0x0000);
    lcd_write_reg(0xd503, 0x006b);
    lcd_write_reg(0xd504, 0x0000);
    lcd_write_reg(0xd505, 0x0084);
    lcd_write_reg(0xd506, 0x0000);
    lcd_write_reg(0xd507, 0x009c);
    lcd_write_reg(0xd508, 0x0000);
    lcd_write_reg(0xd509, 0x00b1);
    lcd_write_reg(0xd50a, 0x0000);
    lcd_write_reg(0xd50b, 0x00D9);
    lcd_write_reg(0xd50c, 0x0000);
    lcd_write_reg(0xd50d, 0x00fd);
    lcd_write_reg(0xd50e, 0x0001);
    lcd_write_reg(0xd50f, 0x0038);
    lcd_write_reg(0xd510, 0x0001);
    lcd_write_reg(0xd511, 0x0068);
    lcd_write_reg(0xd512, 0x0001);
    lcd_write_reg(0xd513, 0x00b9);
    lcd_write_reg(0xd514, 0x0001);
    lcd_write_reg(0xd515, 0x00fb);
    lcd_write_reg(0xd516, 0x0002);
    lcd_write_reg(0xd517, 0x0063);
    lcd_write_reg(0xd518, 0x0002);
    lcd_write_reg(0xd519, 0x00b9);
    lcd_write_reg(0xd51a, 0x0002);
    lcd_write_reg(0xd51b, 0x00bb);
    lcd_write_reg(0xd51c, 0x0003);
    lcd_write_reg(0xd51d, 0x0003);
    lcd_write_reg(0xd51e, 0x0003);
    lcd_write_reg(0xd51f, 0x0046);
    lcd_write_reg(0xd520, 0x0003);
    lcd_write_reg(0xd521, 0x0069);
    lcd_write_reg(0xd522, 0x0003);
    lcd_write_reg(0xd523, 0x008f);
    lcd_write_reg(0xd524, 0x0003);
    lcd_write_reg(0xd525, 0x00a4);
    lcd_write_reg(0xd526, 0x0003);
    lcd_write_reg(0xd527, 0x00b9);
    lcd_write_reg(0xd528, 0x0003);
    lcd_write_reg(0xd529, 0x00c7);
    lcd_write_reg(0xd52a, 0x0003);
    lcd_write_reg(0xd52b, 0x00c9);
    lcd_write_reg(0xd52c, 0x0003);
    lcd_write_reg(0xd52d, 0x00cb);
    lcd_write_reg(0xd52e, 0x0003);
    lcd_write_reg(0xd52f, 0x00cb);
    lcd_write_reg(0xd530, 0x0003);
    lcd_write_reg(0xd531, 0x00cb);
    lcd_write_reg(0xd532, 0x0003);
    lcd_write_reg(0xd533, 0x00cc);

    lcd_write_reg(0xd600, 0x0000);
    lcd_write_reg(0xd601, 0x005d);
    lcd_write_reg(0xd602, 0x0000);
    lcd_write_reg(0xd603, 0x006b);
    lcd_write_reg(0xd604, 0x0000);
    lcd_write_reg(0xd605, 0x0084);
    lcd_write_reg(0xd606, 0x0000);
    lcd_write_reg(0xd607, 0x009c);
    lcd_write_reg(0xd608, 0x0000);
    lcd_write_reg(0xd609, 0x00b1);
    lcd_write_reg(0xd60a, 0x0000);
    lcd_write_reg(0xd60b, 0x00d9);
    lcd_write_reg(0xd60c, 0x0000);
    lcd_write_reg(0xd60d, 0x00fd);
    lcd_write_reg(0xd60e, 0x0001);
    lcd_write_reg(0xd60f, 0x0038);
    lcd_write_reg(0xd610, 0x0001);
    lcd_write_reg(0xd611, 0x0068);
    lcd_write_reg(0xd612, 0x0001);
    lcd_write_reg(0xd613, 0x00b9);
    lcd_write_reg(0xd614, 0x0001);
    lcd_write_reg(0xd615, 0x00fb);
    lcd_write_reg(0xd616, 0x0002);
    lcd_write_reg(0xd617, 0x0063);
    lcd_write_reg(0xd618, 0x0002);
    lcd_write_reg(0xd619, 0x00b9);
    lcd_write_reg(0xd61a, 0x0002);
    lcd_write_reg(0xd61b, 0x00bb);
    lcd_write_reg(0xd61c, 0x0003);
    lcd_write_reg(0xd61d, 0x0003);
    lcd_write_reg(0xd61e, 0x0003);
    lcd_write_reg(0xd61f, 0x0046);
    lcd_write_reg(0xd620, 0x0003);
    lcd_write_reg(0xd621, 0x0069);
    lcd_write_reg(0xd622, 0x0003);
    lcd_write_reg(0xd623, 0x008f);
    lcd_write_reg(0xd624, 0x0003);
    lcd_write_reg(0xd625, 0x00a4);
    lcd_write_reg(0xd626, 0x0003);
    lcd_write_reg(0xd627, 0x00b9);
    lcd_write_reg(0xd628, 0x0003);
    lcd_write_reg(0xd629, 0x00c7);
    lcd_write_reg(0xd62a, 0x0003);
    lcd_write_reg(0xd62b, 0x00c9);
    lcd_write_reg(0xd62c, 0x0003);
    lcd_write_reg(0xd62d, 0x00cb);
    lcd_write_reg(0xd62e, 0x0003);
    lcd_write_reg(0xd62f, 0x00cb);
    lcd_write_reg(0xd630, 0x0003);
    lcd_write_reg(0xd631, 0x00cb);
    lcd_write_reg(0xd632, 0x0003);
    lcd_write_reg(0xd633, 0x00cc);

    lcd_write_reg(0xba00, 0x0024);
    lcd_write_reg(0xba01, 0x0024);
    lcd_write_reg(0xba02, 0x0024);

    lcd_write_reg(0xb900, 0x0024);
    lcd_write_reg(0xb901, 0x0024);
    lcd_write_reg(0xb902, 0x0024);

    lcd_write_reg(0xf000, 0x0055);
    lcd_write_reg(0xf001, 0x00aa);
    lcd_write_reg(0xf002, 0x0052);
    lcd_write_reg(0xf003, 0x0008);
    lcd_write_reg(0xf004, 0x0000);

    lcd_write_reg(0xb100, 0x00cc);
    lcd_write_reg(0xB500, 0x0050);

    lcd_write_reg(0xbc00, 0x0005);
    lcd_write_reg(0xbc01, 0x0005);
    lcd_write_reg(0xbc02, 0x0005);

    lcd_write_reg(0xb800, 0x0001);
    lcd_write_reg(0xb801, 0x0003);
    lcd_write_reg(0xb802, 0x0003);
    lcd_write_reg(0xb803, 0x0003);

    lcd_write_reg(0xbd02, 0x0007);
    lcd_write_reg(0xbd03, 0x0031);
    lcd_write_reg(0xbe02, 0x0007);
    lcd_write_reg(0xbe03, 0x0031);
    lcd_write_reg(0xbf02, 0x0007);
    lcd_write_reg(0xbf03, 0x0031);

    lcd_write_reg(0xff00, 0x00aa);
    lcd_write_reg(0xff01, 0x0055);
    lcd_write_reg(0xff02, 0x0025);
    lcd_write_reg(0xff03, 0x0001);

    lcd_write_reg(0xf304, 0x0011);
    lcd_write_reg(0xf306, 0x0010);
    lcd_write_reg(0xf308, 0x0000);

    lcd_write_reg(0x3500, 0x0000);
    lcd_write_reg(0x3600, 0x0060);
    lcd_write_reg(0x3A00, 0x0005);
    //Display On
    lcd_write_cmd_byte(0x2900);
    // Out sleep
    lcd_write_cmd_byte(0x1100);
    // Write continue
    lcd_write_cmd_byte(0x2C00);
}

uint16_t BBF[100] = {0x00, 0xffff, 0x00, 0xffff, 0x00, 0xffff, 0x00, 0xffff, 0x00, 0xffff};

void lcd_edma_attach(void)
{
    I2S0.clkm_conf.clkm_div_num = 16;
    I2S0.sample_rate_conf.tx_bck_div_num = 2;
    uint8_t *pbuf = (uint8_t *)heap_caps_calloc(1, sizeof(uint16_t) * 800 * 480, MALLOC_CAP_SPIRAM);
    if(pbuf == NULL) {
        printf("pbuf NULL\n");
        return;
    }
    memset(pbuf, 0, sizeof(uint16_t) * 800 * 480);
    uint8_t *_buf = pbuf;
    for(int i = 0; i < 250; i++) {
        __dma[i].size = 3*1024;
        __dma[i].length = 3*1024;
        __dma[i].buf = _buf;
        __dma[i].eof = 1;
        __dma[i].empty = &__dma[(i+1) % 250];
        _buf += 3*1024;
    }
    I2S0.out_link.addr = ((uint32_t)&__dma[0]) & 0xfffff;
    I2S0.out_link.start = 1;
    I2S0.fifo_conf.dscr_en = 1;
    I2S0.conf.tx_start = 1;
}

void lcd_init(void)
{
    nt35510_init();
    printf("lcd init done\n");
    lcd_fill_screen(0x3c1c);
    vTaskDelay(3000/portTICK_PERIOD_MS);
    printf("start\n");
    vTaskDelay(3000/portTICK_PERIOD_MS);
    // i2s_lcd_ena_fram_buf();
    // lcd_draw_bmp(gImage_pic);
    lcd_edma_attach();
    // while(1) {
    //     printf("%x\n", I2S0.int_raw.val);
    //     vTaskDelay(1000/portTICK_PERIOD_MS);
    // }
    // int k = 0;
    // while(1) {
    //      lcd_fill_screen(k);
    //    // lcd_fill_box(10, 50, 10, 80, 0xc999);
    //     k++;
    //     if(k % 60 == 0) {
    //         printf("..\n");
    //     }
    // }
}