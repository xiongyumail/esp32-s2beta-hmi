# Hello World Example

Starts a FreeRTOS task to print "Hello World"

See the README.md file in the upper level 'examples' directory for more information about examples.

## LCD子板

IO数量: 18 + 2(I2C)

HMI Demo使用了16位并行模式

LCD Touch 使用I2C接口，可以复用 2 IO

原理图中LCD_RST和BL_CTR要注意保持拉高，其它引脚可留空

![LCD](LCD_Socket.png)

```
#define  WR  GPIO_NUM_34
#define  RS  GPIO_NUM_1

#define  D0  GPIO_NUM_35
#define  D1  GPIO_NUM_37
#define  D2  GPIO_NUM_36
#define  D3  GPIO_NUM_39
#define  D4  GPIO_NUM_38
#define  D5  GPIO_NUM_41
#define  D6  GPIO_NUM_40
#define  D7  GPIO_NUM_45

#define  D8   GPIO_NUM_21
#define  D9   GPIO_NUM_18
#define  D10  GPIO_NUM_17
#define  D11  GPIO_NUM_16
#define  D12  GPIO_NUM_15
#define  D13  GPIO_NUM_14
#define  D14  GPIO_NUM_13
#define  D15  GPIO_NUM_12

#define SDA (GPIO_NUM_3)
#define SCL (GPIO_NUM_5)
```