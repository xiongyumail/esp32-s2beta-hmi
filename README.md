# _ESP32-S2Beta-HMI_

_HMI evaluation suite for new low power, low cost and secure Internet of Things chip ESP32-S2. ESP32-S2 chip is equipped with Xtensa 32-bit LX7 single-core processor.
Working frequency up to 240 MHz, providing a wealth of peripheral interfaces, including SPI, I2S, UART, I2C, LED PWM, PCNT, LCD interface, Camera interface, ADC, DAC,
Touch sensors, temperature sensors and up to 43 GPIOs. Supports high-resolution LCD displays and Camera Sensor, as well as a variety of extension options.
Perfectly supports open source GUI LittleVGL, providing a large number of rich and easy-to-use GUI components and design examples._


* Development board Structure

  ![Board](data/Structure.png)

  * High Definition Resolution: 800x480 4.3 inch

  * Multiple interfaces: 8080, 6800, SPI

  * Easy to use and beautiful GUI: LittleVGL

  * Powerful and rich graphics components: buttons, charts, lists, sliders, images, etc.

  * A large number of sensor drivers: WS2812, MPU6050, HTS221, BH170, ES8311, OV2640 etc.

  * Multiple GUI examples: RGB LED Color picker, MPU6050 Data Curve, A simple piano, Simple Terminal, etc.

## Contents

* Directory tree

    ```
    ├── add_path.sh
    ├── data
    ├── project
    │   └── hmi
    ├── README.md
    └── tools
        ├── esp-idf
        └── xtensa-esp32s2-elf
    ```

    * project

      Development examples

    * tools

      * ESP32-S2 sdk

      * ESP32-S2 toolchain

    * Data

      * Schematic diagram

      * Misc

## How to use

You can follow the steps below to set up the development environment, or directly download the release version of the full environment.

* clone

  ```bash
  git clone --recursive https://gitlab.espressif.cn:6688/yxiong/esp32-s2beta-hmi
  cd esp32-s2beta-hmi
  ```

  * note

    Don't omit `--recursive`, because we use submodule.

* update

  ```bash
  git pull
  git submodule update --init --recursive
  ```

* Get toolchain

  xtensa-esp32s2-elf

  * [Windows](https://dl.espressif.com/dl/toolchains/preview/xtensa-esp32s2-elf-gcc8_2_0-esp32s2-dev-4-g3a626e-win32.zip)
  * [Mac](https://dl.espressif.com/dl/toolchains/preview/xtensa-esp32s2-elf-gcc8_2_0-esp32s2-dev-4-g3a626e-macos.tar.gz)
  * [Linux(64)](https://dl.espressif.com/dl/toolchains/preview/xtensa-esp32s2-elf-gcc8_2_0-esp32s2-dev-4-g3a626e-linux-amd64.tar.gz)
  * [Linux(32)](https://dl.espressif.com/dl/toolchains/preview/xtensa-esp32s2-elf-gcc8_2_0-esp32s2-dev-4-g3a626e-linux-i686.tar.gz)

* Install toolchain

  Example: Linux(64)

  ```bash
  wget https://dl.espressif.com/dl/toolchains/preview/xtensa-esp32s2-elf-gcc8_2_0-esp32s2-dev-4-g3a626e-linux-amd64.tar.gz
  tar zxvf xtensa-esp32s2-elf-gcc8_2_0-esp32s2-dev-4-g3a626e-linux-amd64.tar.gz -C tools/
  rm xtensa-esp32s2-elf-gcc8_2_0-esp32s2-dev-4-g3a626e-linux-amd64.tar.gz
  ```

* Add environment variables

  ```bash
  . add_path.sh
  ```
  * note

    Don't omit `"."`

## Appendix

### Schematic

* [Main Board](data/SCH_ESP32-S2-HMI_V1_20190528A.pdf)

* [LCD Board](data/SCH_ESP32-S2-HMI-LCD_V1_0_20190611A.pdf)

* [Audio Board](data/audio.pdf)

* [Touch Board](data/TOUCH.pdf)

### Datasheet

* [ESP32-S2](data/esp32-s2_datasheet_en.pdf)

* [LCD-NT35510](data/LCD_NT35510.pdf)

* [CAM-OV2640](data/LCD_NT35510.pdf)

