#include <stdio.h>
#include "driver/i2c.h"
#include "fram_cfg.h"

#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */


int i2c_master_port = 1;

#define SCL   GPIO_NUM_16
#define SDA   GPIO_NUM_15

uint8_t ESP_SLAVE_ADDR = 0x0;

struct reg_raw {
  uint8_t reg;
  uint8_t cfg;
};

void sccb_bus_init(void)
{
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = SDA;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = SCL;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 200000;
  i2c_param_config(i2c_master_port, &conf);
  i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

int sccb_read_reg(uint8_t reg, uint8_t* data)
{
  esp_err_t ret = ESP_FAIL;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( ESP_SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) return -1;
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( ESP_SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret == ESP_OK ? 0 : -1;
}

int sccb_write_reg(uint8_t reg, uint8_t data)
{
  esp_err_t ret = ESP_FAIL;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( ESP_SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret == ESP_OK ? 0 : -1;
}

int sccb_slave_prob(void)
{
  uint8_t slave_addr = 0x0;
  sccb_bus_init();
  while (slave_addr < 0x7f) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slave_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if ( ret == ESP_OK) {
      ESP_SLAVE_ADDR = slave_addr;
      printf("slave_addr 0x%x\n", slave_addr);
      return 0;
    }
    slave_addr++;
  }
  return -1;
}

struct reg_raw uxga_cfg_tbl[] =
{
  {0xff, 0x00},
  {0x2c, 0xff},
  {0x2e, 0xdf},
  {0xff, 0x01},
  {0x3c, 0x32},
  {0x11, 0x81},
  // {0x11, 0x81},
  {0x09, 0x02},
  {0x04, 0xf8},//水平镜像,垂直翻转
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
  {0xDA, 0x09},
  {0xD7, 0x03},
  {0xDF, 0x02},
  {0x33, 0xa0},
  {0x3C, 0x00},
  {0xe1, 0x67},
  {0xff, 0x01},
  {0xe0, 0x00},
  {0xe1, 0x00},
  {0xe5, 0x00},
  {0xd7, 0x00},
  {0xda, 0x00},
  {0xe0, 0x00},
};

void camera_set_pclk(void)
{

}

void camera_reg_cfg(void)
{
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

  // uint8_t outw = 320 / 4;
  // uint8_t outh = 240 / 4;
  uint8_t outw = FRAM_WIDTH / 4;
  uint8_t outh = FRAM_HIGH / 4;
  sccb_write_reg(0XFF, 0X00);
  sccb_write_reg(0XE0, 0X04);
  sccb_write_reg(0X5A, outw & 0XFF); //设置OUTW的低八位
  sccb_write_reg(0X5B, outh & 0XFF); //设置OUTH的低八位
  temp = (outw >> 8) & 0X03;
  temp |= (outh >> 6) & 0X04;
  sccb_write_reg(0X5C, temp);      //设置OUTH/OUTW的高位
  sccb_write_reg(0XE0, 0X00);

#if 0
  uint8_t reg;
  sccb_write_reg(0XFF, 0X01);
  sccb_read_reg(0X12, &reg);
  reg &= ~(1 << 1);
  reg |= 1 << 1;
  sccb_write_reg(0X12, reg);

  sccb_write_reg(0XFF, 0X00);

  sccb_write_reg(0Xc7, 0X40);
  sccb_write_reg(0Xcc, 0X5e);
  sccb_write_reg(0Xcd, 0X41);
  sccb_write_reg(0Xce, 0X54);
#endif
  sccb_write_reg(0XFF, 0X00);

#if 0
  /*  YUYV   */
  sccb_read_reg(0xda, &temp);
  temp &= 0xfe;
  sccb_write_reg(0xda, temp);
  sccb_read_reg(0xc2, &temp);
  temp &= 0xef;
  sccb_write_reg(0xc2, temp);
#endif
#if 0
  /*  YVYU   */
  sccb_read_reg(0xda, &temp);
  temp &= 0xfe;
  sccb_write_reg(0xda, temp);
  sccb_read_reg(0xc2, &temp);
  temp |= 0x10;
  sccb_write_reg(0xc2, temp);
#endif
#if  1
  /*  VYUY   */
  sccb_read_reg(0xda, &temp);
  temp |= 0x00;
  sccb_write_reg(0xda, temp);
  sccb_read_reg(0xc2, &temp);
  temp &= 0xef;
  sccb_write_reg(0xc2, temp);
#endif
#if 0
  /*  UYVY   */
  // sccb_write_reg(0xda, 0x08);
  sccb_read_reg(0xc2, &temp);
  temp &= 0xf0;
  temp |= 0x16;
  sccb_write_reg(0xc2, temp);
  sccb_read_reg(0xda, &temp);
#endif
  sccb_write_reg(0x00, 0X00);
}