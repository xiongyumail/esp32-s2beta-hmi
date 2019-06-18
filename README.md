# ESP32-S2Beta-HMI

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