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

All details related to the project can be found at 
[docs.t3gemstone.org/en/projects/ardupilot](https://docs.t3gemstone.org/en/projects/ardupilot).
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
VEHICLE=plane task build
```

## Board Setup

Run following command in the host PC. Board setup is automated via SSH connections.

```bash
VEHICLE=plane task build board-setup
```

You need to reboot the board for overlay changes to take effect. After the reboot, ArduPilot should start automatically 
and on-board green LED located to the right of the HDMI port should be blinking.
Check ArduPilot docs to learn
more about [LEDs meaning](https://ardupilot.org/copter/docs/common-leds-pixhawk.html#boards-with-1-or-2-notify-leds).

## Firmware Upload

If you have done Board Setup once and only want to upload new firmware then run the following commands in the host PC:

```bash
VEHICLE=plane task build board-upload
```

## Software-in-the-Loop (SITL)

This repository includes a complete SITL environment for ArduPilot development and testing, orchestrated through Docker Compose. The setup integrates ArduPilot SITL with MissionPlanner and Gazebo simulation for realistic vehicle testing.

### Architecture

The SITL environment consists of three containerized services:

- **ArduPilot SITL**: Flight controller simulation running your vehicle firmware
- **MissionPlanner**: Ground control station for mission planning and monitoring
- **Gazebo**: 3D physics simulation environment for realistic vehicle dynamics

### Quick Start

Start the SITL environment:
```bash
task sitl-start
```

Stop the SITL environment:
```bash
task sitl-stop
```

### Configuration

SITL parameters are configured in the Taskfile:

```yaml
  SITL_LOCATION: --location ARDUCOPTER_AUTOTEST
  SITL_VEHICLE: --vehicle ArduPlane
  SITL_FRAME: # --frame
  SITL_PARAM: --add-param-file skywalker_x8_quad.param
  SITL_WORLD: skywalker_x8_quad_runway.sdf
```

#### Available Configuration Options

**SITL_LOCATION**: Starting location for the simulated vehicle
- Find available locations in [ArduPilot locations.txt](https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/locations.txt)
- Examples: `ARDUCOPTER_AUTOTEST`, `KSFO`, `CMAC`

**SITL_VEHICLE**: Vehicle type to simulate
- Find available vehicles and their parameters in [SITL_Models documentation](https://github.com/ArduPilot/SITL_Models/tree/master/Gazebo/docs)
- Examples: `ArduPlane`, `ArduCopter`, `ArduSub`, `Rover`

**SITL_FRAME**: Frame type for vehicle configuration
- Find available frame types in [ArduPilot documentation](https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/pysim/vehicleinfo.py)
- Examples: `gazebo-iris`, `gazebo-zephyr`

**SITL_PARAM**: Parameter file for vehicle configuration
- Find available parameter files in [SITL_Models documentation](https://github.com/ArduPilot/SITL_Models/tree/master/Gazebo/docs)
- Examples: `skywalker_x8_quad.param`, `skywalker_x8.param`

**SITL_WORLD**: Gazebo world file for simulation environment
- Find available worlds in [SITL_Models worlds directory](https://github.com/ArduPilot/SITL_Models/tree/master/Gazebo/worlds)
- Examples: `skywalker_x8_quad_runway.sdf`, `skywalker_x8_runway.sdf`

For some vehicles you need to specify both SITL_FRAME and SITL_PARAM. Some vehicles use only one and some omit both.

### Connecting to SITL

On Mission Planner, the connection are set up using the drop down boxes in the upper right portion of the screen.
Select UDP from that list and click "Connect". It asks for UDP port, accept the default value which is **14550**.
Mission Planner will connect to SITL instance. 

Mission Planner is a Windows native app that runs with Mono runtime on Linux. Sometimes it can become unresponsive.
When it becomes unresponsive you can restart it with `task sitl-restart-mp` command.

### Applying Configuration Changes

After modifying any SITL variables in the Taskfile, restart the environment:

```bash
task sitl-start
```

The Docker Compose configuration will automatically pick up the new environment variables and restart the services with
the updated settings.
