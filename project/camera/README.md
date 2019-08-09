# Hello World Example

Starts a FreeRTOS task to print "Hello World"

See the README.md file in the upper level 'examples' directory for more information about examples.


## Camera子板

IO数量: 12 + 2(I2C)

HMI Demo使用了GPIO0作为Camera时钟输出， 一定程度会影响下载

Camera使用的I2C最好能独立提供

![Camera](Camera_Socket.png)

```
#define D0 GPIO_NUM_18
#define D2 GPIO_NUM_17
#define D4 GPIO_NUM_8
#define D6 GPIO_NUM_10

#define D1 GPIO_NUM_21
#define D3 GPIO_NUM_7
#define D5 GPIO_NUM_9
#define D7 GPIO_NUM_11

#define XCLK  GPIO_NUM_0
#define PCLK  GPIO_NUM_12
#define VSYNC GPIO_NUM_14
#define HSYNC GPIO_NUM_13

#define SIOC   GPIO_NUM_16
#define SIOD   GPIO_NUM_15
```