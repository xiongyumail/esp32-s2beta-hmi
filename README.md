# ESP32-S2Beta-HMI

ESP32-S2Beta-HMI： HMI evaluation suite for new low power, low cost and secure Internet of Things chip ESP32-S2. ESP32-S2 chip is equipped with Xtensa 32-bit LX7 single-core processor.
Working frequency up to 240 MHz, providing a wealth of peripheral interfaces, including SPI, I2S, UART, I2C, LED PWM, PCNT, LCD interface, Camera interface, ADC, DAC,
Touch sensors, temperature sensors and up to 43 GPIOs. Supports high-resolution LCD displays and Camera Sensor, as well as a variety of extension options.
Perfectly supports open source GUI LittleVGL, providing a large number of rich and easy-to-use GUI components and design examples.

Key features:
* High Definition Resolution: 800x480 4.3 inch
* Multiple interfaces: 8080, 6800, SPI
* Easy to use and beautiful GUI: LittleVGL
* Powerful and rich graphics components: buttons, charts, lists, sliders, images, etc.
* A large number of sensor drivers: WS2812, MPU6050, HTS221, BH170, ES8311, OV2640 etc.
* Multiple GUI examples: RGB LED Color picker, MPU6050 Data Curve, A simple piano, Simple Terminal, etc.

esptool 添加gitlab仓库

idf.py -DIDF_TARGET=esp32s2beta build flash

I2C:
SW5 4 TH_GPIO3 EXT_GPIO3
SW5 6 TH_GPIO5 EXT_GPIO5

# _ESP32-S2Beta-HMI_

## Contents

* Directory tree

    ```

    ```

    * project

    * tools

      * script

      * sdk

      * toolchain

## How to use

You can follow the steps below to set up the development environment, or directly download the release version of the full environment.

* clone

  ```bash
  ```

  * note

    Don't omit `--recursive`, because we use submodule.

* update

  ```bash
  git pull
  git submodule update --init --recursive
  ```

* Add environment variables

  ```bash
  . add_path.sh
  ```
  * note

    Don't forget `"."`

* Install toolchain

  * ESP32-S2Beta

    * Ubuntu/Debian/Kali/Arch

      ```bash
      ```

## Release

* download

  ```bash
  ```
* Add environment variables

  ```bash
  . add_path.sh
  ```
  * note

    Don't forget `"."`

## License

## Contributing