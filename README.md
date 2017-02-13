# whmnet

The whmnet project consists in:
- A wireless sensor network which performs temperature, atmospheric pressure and relative humidity measurements
- A gateway which receives the measurements and pass them to a Raspberry Pi
- A Raspberry Pi which is connected to the Internet, posting measurements on a webserver
- A web server hosting the measurements database and a web application to display the date

# The Sensor

The wireless sensor is a custom PCB based on:
- A STM32L0 ARM-Cortex M0 microcontroler which gathers data from sensors and send them over the air
- SHT21 Relative Humdity Sensor - I2C
- MS5637 Barometric Pressure Sensor - I2C
- CC1101 868MHz transceiver from Texas Instruments - SPI
- On-board 868MHz ceramic antenna and IPC device (balun + filter + match)
- All of this is powered by a CR2450 lithium coin cell downconverted to 2.1V by TPS62730 ultra-low-power DC-DC converter

The wireless sensor performs a measurement every 5 minutes and goes back to low-power mode between each measurement, achieving an ultra-low-power average consumption of 5µA leading to several months of battery life.

The PCB design files are available in the [SensorBoard](./SensorBoard/) directory. It has been designed with Altium Designer.

The MCU firmware is available in the [Sensor](./Sensor/) directory. [SW4STM32](http://www.openstm32.org) Eclipse based IDE from AC6 has been used. The toolchain is GCC-based, and the (free) IDE is part of the STMicroelectronics Open Development Environment. BSP has been generated with [STM32CubeMX](http://www.st.com/en/development-tools/stm32cubemx.html) tool and the [STM32CubeL0](http://www.st.com/en/embedded-software/stm32cubel0.html) open-source library is used.

# The Gateway

The gateway consists in:
- A custom board for wireless reception
- A Raspberry Pi 2 model B

## Gateway RF Board

The custom board includes:
- An STM32L0 microcontroler
- A CC1120 RF transceiver for wireless communication
- A green LED indicating radio packet Rx

The STM32L0 reports the measurements from wireless sensors to the Raspberry Pi using the serial port.

The PCB design files are available in the [GatewayBoard](./GatewayBoard/) directory.
The MCU firmware of the Gateway Board is available in the [Gateway](./Gateway/) directory.

## Raspberry Pi

The Raspberry Pi:
- Receives measurements from the custom board over the UART
- Check data validity (by controlling CRC values)
- Push the data on a webserver on the internet

Failures, hangs, loss of internet connection, etc. are all taken care of by dedicated scripts and watchdogs which perform data backup, retries, process monitoring and eventually reboots.

# The webserver

The webserver, hosted on the internet:
- stores data coming from the Gateway
- run a web application for data display



