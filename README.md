<p align="center">
    <picture>
        <source media="(prefers-color-scheme: dark)" srcset=".meta/logo-dark.png" width="40%" />
        <source media="(prefers-color-scheme: light)" srcset=".meta/logo-light.png" width="40%" />
        <img alt="T3 Foundation" src=".meta/logo-light.png" width="40%" />
    </picture>
</p>

# Gemstone ArduPilot

[![T3 Foundation](./.meta/t3-foundation.svg)](https://www.t3vakfi.org/en) [![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## What is it?

This repository contains the version of the open-source autopilot called ArduPilot that runs on
T3 Gemstone development boards.

All details related to the project can be found at https://docs.t3gemstone.org/en/projects/ardupilot. 
Below, only a summary of how to perform the building is provided.

## Build

Run all commands in the host PC. ArduPilot is cross-compiled for T3 Gemstone boards using T3 Gemstone Toolchains. 
You can select which vehicle you want to compile by changing the `VEHICLE` variable defined inside `Taskfile.yml`.

##### 1. Clone the project

```bash
git clone https://github.com/t3gemstone/ardupilot
cd ardupilot
git submodule update --init --recursive
```

##### 2. Install Taskfile

```bash
sh -c "$(curl --location https://taskfile.dev/install.sh)" -- -d -b /usr/local/bin
```

##### 3. Compile the project

```bash
task build
```

## Board Setup

Run following command in the host PC. Board setup is automated via SSH connections.

```bash
task board-setup
```

You need to reboot the board for overlay changes to take effect. After reboot, ArduPilot should start automatically 
and on-board green LED located to the right of HDMI port should be blinking.
Check ArduPilot docs to learn
more about [LEDs meaning](https://ardupilot.org/copter/docs/common-leds-pixhawk.html#boards-with-1-or-2-notify-leds).

## Firmware Upload

If you have done Board Setup once and only want to upload new firmware then run the following commands in the host PC:

```bash
task build
task board-upload
```

## QGroundControl (QGC)

You can establish MAVLink connection between ArduPilot and QGC via UDP or Serial.
By default UDP broadcast at 192.168.7.255 subnet and 14550 port is used for `serial0` and UART-MAIN1 (`/dev/ttyS3`)
is used for `serial1`. You can change them by editing the `/opt/gemstone/ardupilot/ardupilot.env` file.

### UDP Link

Connect T3-GEM-O1 to your PC via USB Type-C cable. T3-GEM-O1 utilizes USB Gadget API to achieve Ethernet-over-USB.
After board is fully booted, a new Ethernet interface should appear on your PC.
QGC will automatically connect to ArduPilot via UDP. MAVLink messages should arrive now and you should be able to see
the status of the vehicle.

### Serial Link

Connect T3-GEM-O1 to your PC via USB Type-C cable. Connect RX, TX and GND pins of USB-to-TTL adapter to respective
GPIO pins. After you insert the adapter to your PC, a new TTY device should be created.
Open `Application Settings -> Comm Links` menu in QGC. Click the "Add" button.
Select the right serial port and 57600 baud rate.
After saving the configuration, click "Connect" and exit "Application Settings".
MAVLink messages should arrive now and you should be able to see the status of the vehicle.

## GPIO Pinout

Following table shows the function of each pin in the GPIO header after applying device-tree overlays.

| FUNCTION                 | PINS                  | PINS                  | FUNCTION                       |
|-------------------------:|:---------------------:|:---------------------:|:-------------------------------|
| 3v3 Power                | **3v3 Power**         | **5v Power**          | 5v Power                       |
| I2C-MCU0 SDA (GPS)       | **GPIO-2 (SYS_506)**  | **5v Power**          | 5v Power                       |
| I2C-MCU0 SCL (GPS)       | **GPIO-3 (SYS_505)**  | **GND**               | GND                            |
| UART-MAIN6 RX (GPS)      | **GPIO-4 (SYS_439)**  | **GPIO-14 (SYS_342)** | UART-MAIN1 TX (Telemetry)      |
| GND                      | **GND**               | **GPIO-15 (SYS_341)** | UART-MAIN1 RX (Telemetry)      |
| UART-MAIN6 TX (GPS)      | **GPIO-17 (SYS_336)** | **GPIO-18 (SYS_339)** | PWM-ECAP2 (RCOut-3)            |
|                          | **GPIO-27 (SYS_434)** | **GND**               | GND                            |
|                          | **GPIO-22 (SYS_442)** | **GPIO-23 (SYS_495)** |                                |
| 3v3 Power                | **3v3 Power**         | **GPIO-24 (SYS_498)** | UART-WKUP0 TX (SBUS RC Input)  |
| SPI-MCU0 MOSI            | **GPIO-10 (SYS_491)** | **GND**               | GND                            |
| SPI-MCU0 MISO            | **GPIO-9 (SYS_492)**  | **GPIO-25 (SYS_443)** |                                |
| SPI-MCU0 SCLK            | **GPIO-11 (SYS_490)** | **GPIO-8 (SYS_488)**  | SPI-MCU0 CS0                   |
| GND                      | **GND**               | **GPIO-7 (SYS_497)**  | UART-WKUP0 RX (SBUS RC Input)  |
| I2C-WKUP0 SDA (Reserved) | **GPIO-0**            | **GPIO-1**            | I2C-WKUP0 SCL (Reserved)       |
| PWM-0A (RCOut-4)         | **GPIO-5 (SYS_343)**  | **GND**               | GND                            |
| PWM-1A (RCOut-6)         | **GPIO-6 (SYS_345)**  | **GPIO-12 (SYS_344)** | PWM-ECAP0 (RCOut-1)            |
| PWM-1B (RCOut-7)         | **GPIO-13 (SYS_346)** | **GND**               | GND                            |
|                          | **GPIO-19 (SYS_340)** | **GPIO-16 (SYS_335)** | PWM-ECAP1 (RCOut-2)            |
| Buzzer                   | **GPIO-26 (SYS_437)** | **GPIO-20 (SYS_338)** |                                |
| GND                      | **GND**               | **GPIO-21 (SYS_337)** | FAN                            |

- PWM-2A (RCOut-5): PWM pin of the 4-pin FAN header on the board

Linux paths of each peripheral are listed below.

```
UART-WKUP0:   /dev/ttyS0
UART-MAIN1:   /dev/ttyS3
UART-MAIN6:   /dev/ttyS6

I2C-MCU0:     /dev/i2c-1

SPI-MCU0 CS0: /dev/spidev0.0

PWM-ECAP0:    /sys/class/pwm/pwmchip0/pwm0
PWM-ECAP1:    /sys/class/pwm/pwmchip1/pwm0
PWM-ECAP2:    /sys/class/pwm/pwmchip2/pwm0
PWM-0A:       /sys/class/pwm/pwmchip3/pwm0
PWM-1A:       /sys/class/pwm/pwmchip5/pwm0
PWM-1B:       /sys/class/pwm/pwmchip5/pwm1
PWM-2A:       /sys/class/pwm/pwmchip7/pwm0

FAN:          /sys/class/thermal/cooling_device0
```
