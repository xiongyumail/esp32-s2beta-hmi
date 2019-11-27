#include "sccb.h"
#include "driver/i2c.h"

int i2c_master_port = 1;

#define SCL   GPIO_NUM_16
#define SDA   GPIO_NUM_15

#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

//初始化SCCB接口 
void SCCB_Init(void)
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

//写寄存器
//返回值:0,成功;1,失败.
uint8_t SCCB_WR_Reg(uint8_t reg, uint8_t data)
{
    esp_err_t ret = ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SCCB_ID | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? 0 : 1;
}
//读寄存器
//返回值:读到的寄存器值
uint8_t SCCB_RD_Reg(uint8_t reg)
{
    uint8_t val = 0;
    esp_err_t ret = ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SCCB_ID | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return -1;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SCCB_ID | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &val, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        printf("SCCB_RD_Reg error\n");
    }

    return val;
}
















