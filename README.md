# STM32F407x-MCU-Driver-Development

![driver development](https://github.com/HalimMansour/STM32F407x-MCU-Driver-Development/assets/122712424/7b3018f0-7398-42c6-88ae-1c64e3872855)

This repository contains the driver APIs for GPIO, SPI, I2C, and UART for the STM32F407x microcontroller unit (MCU). The provided APIs enable seamless integration and control of these peripherals in your STM32F407x MCU-based projects.

## Features

- **GPIO Driver API:** The GPIO driver API provides functions for initializing GPIO pins, configuring their modes (input, output, alternate function), setting pull-up/pull-down resistors, and reading or writing pin values.
  
![GPIO Driver API Req](https://github.com/HalimMansour/STM32F407x-MCU-Driver-Development/assets/122712424/2e655305-6925-449d-a103-2322d4c998e1)

- **SPI Driver API:** The SPI driver API allows you to initialize and configure the SPI peripheral, set the communication parameters (data size, clock polarity, clock phase), and perform data transmission and reception using the SPI protocol.

![SPI Driver API Req](https://github.com/HalimMansour/STM32F407x-MCU-Driver-Development/assets/122712424/795ffeec-fa1e-4ce3-861b-7afd711cfc70)

- **I2C Driver API:** The I2C driver API provides functions to initialize and configure the I2C peripheral, set the clock speed, perform master or slave mode operations, and transmit or receive data using the I2C protocol.

- **UART Driver API:** The UART driver API enables serial communication using the UART peripheral. It provides functions to initialize and configure the UART, set baud rate, data size, parity, and stop bits, as well as transmit and receive data via the UART interface.
