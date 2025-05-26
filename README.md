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

All details related to the project can be found at https://docs.t3gemstone.org/en/projects/ardupilot. Below, only a summary of how to perform the building is provided.

## Build

Run following commands in the host PC:

##### 1. Clone the project

```bash
git clone https://github.com/t3gemstone/ardupilot
cd ardupilot
git checkout pr-t3-gem-o1-linux-board
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

## Install

Run following commands in the host PC:

```bash
SSH_USER=gemstone
SSH_IP_ADDR=192.168.7.2
ssh "$SSH_USER@$SSH_IP_ADDR" "mkdir -p /home/gemstone/ardupilot"
scp -r gemstone/services "$SSH_USER@$SSH_IP_ADDR":ardupilot
scp build/t3-gem-o1/bin/arducopter gemstone/{ardupilot.env,ardupilot.parm} "$SSH_USER@$SSH_IP_ADDR":ardupilot
```

Run following commands in the `t3-gem-o1` board:

```bash
# Copy arducopter binary and configuration files
sudo install --directory --owner $USER --group $USER /opt/gemstone/ardupilot
cp $HOME/ardupilot/{arducopter,ardupilot.env,ardupilot.parm} /opt/gemstone/ardupilot

# Copy systemd service unit file and enable the service
sudo cp $HOME/ardupilot/services/* /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable arducopter

# Update device-tree overlays
sudo sed -i 's/^overlays=.*$/overlays=k3-am67a-t3-gem-o1-spidev0.dtbo k3-am67a-t3-gem-o1-spi0-1cs.dtbo k3-am67a-t3-gem-o1-i2c1-400000.dtbo k3-am67a-t3-gem-o1-uart-ttys0.dtbo k3-am67a-t3-gem-o1-uart-ttys6.dtbo k3-am67a-t3-gem-o1-gpio-fan.dtbo k3-am67a-t3-gem-o1-pwm-ecap0-gpio12.dtbo k3-am67a-t3-gem-o1-pwm-ecap1-gpio16.dtbo k3-am67a-t3-gem-o1-pwm-ecap2-gpio18.dtbo k3-am67a-t3-gem-o1-pwm-epwm0-gpio5.dtbo k3-am67a-t3-gem-o1-pwm-epwm1-gpio6-gpio13.dtbo/' /boot/uEnv.txt

# Reboot is needed for changes to take effect
sudo reboot
```

After reboot, Arducopter should start automatically and on-board green LED should be blinking.
Check ArduPilot docs to learn 
more about [LEDs meaning](https://ardupilot.org/copter/docs/common-leds-pixhawk.html#boards-with-1-or-2-notify-leds).

## QGroundControl

You can establish MAVLink ground station connection via serial or UDP. By default UART-MAIN1 (`/dev/ttyS3`) is used.
You can change it by editing the `/opt/gemstone/ardupilot/ardupilot.env` file. Check ArduPilot docs to learn more about [serial port configuration options](https://ardupilot.org/copter/docs/common-serial-options.html).

Connect RX, TX and GND pins of USB-to-TTL adapter to respective GPIO pins. After you insert the adapter to host PC, a
new TTY device should be created.

In the QGroundControl interface, open `Application Settings -> General` menu. There are checkboxes under "AutoConnect to the following devices" heading.
Uncheck all of them as they prevent you from connecting via UART.

Now open `Application Settings -> Comm Links` menu. Click the "Add" button. Select the right serial port and 115200 baud rate.
After saving the configuration, click "Connect" and exit "Application Settings". MAVLink messages should arrive now and you
should be able to see the status of the vehicle.

## GPIO Pinout

Following table shows the function of each pin in the GPIO header after applying device-tree overlays.

| FUNCTION                 | PINS                  | PINS                  | FUNCTION                 |
|-------------------------:|:---------------------:|:---------------------:|:-------------------------|
| 3v3 Power                | **3v3 Power**         | **5v Power**          | 5v Power                 |
| I2C-MCU0 SDA             | **GPIO-2 (SYS_506)**  | **5v Power**          | 5v Power                 |
| I2C-MCU0 SCL             | **GPIO-3 (SYS_505)**  | **GND**               | GND                      |
| UART-MAIN6 RX            | **GPIO-4 (SYS_439)**  | **GPIO-14 (SYS_342)** | UART-MAIN1 TX            |
| GND                      | **GND**               | **GPIO-15 (SYS_341)** | UART-MAIN1 RX            |
| UART-MAIN6 TX            | **GPIO-17 (SYS_336)** | **GPIO-18 (SYS_339)** | PWM-ECAP2 (RCOut-3)      |
|                          | **GPIO-27 (SYS_434)** | **GND**               | GND                      |
|                          | **GPIO-22 (SYS_442)** | **GPIO-23 (SYS_495)** |                          |
| 3v3 Power                | **3v3 Power**         | **GPIO-24 (SYS_498)** | UART-WKUP0 TX            |
| SPI-MCU0 MOSI            | **GPIO-10 (SYS_491)** | **GND**               | GND                      |
| SPI-MCU0 MISO            | **GPIO-9 (SYS_492)**  | **GPIO-25 (SYS_443)** |                          |
| SPI-MCU0 SCLK            | **GPIO-11 (SYS_490)** | **GPIO-8 (SYS_488)**  | SPI-MCU0 CS0             |
| GND                      | **GND**               | **GPIO-7 (SYS_497)**  | UART-WKUP0 RX            |
| I2C-WKUP0 SDA (Reserved) | **GPIO-0**            | **GPIO-1**            | I2C-WKUP0 SCL (Reserved) |
| PWM-0A (RCOut-4)         | **GPIO-5 (SYS_343)**  | **GND**               | GND                      |
| PWM-1A (RCOut-6)         | **GPIO-6 (SYS_345)**  | **GPIO-12 (SYS_344)** | PWM-ECAP0 (RCOut-1)      |
| PWM-1B (RCOut-7)         | **GPIO-13 (SYS_346)** | **GND**               | GND                      |
|                          | **GPIO-19 (SYS_340)** | **GPIO-16 (SYS_335)** | PWM-ECAP1 (RCOut-2)      |
| Buzzer                   | **GPIO-26 (SYS_437)** | **GPIO-20 (SYS_338)** |                          |
| GND                      | **GND**               | **GPIO-21 (SYS_337)** | FAN                      |

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
