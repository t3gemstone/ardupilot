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

##### 1. Install Dependencies

```bash
sudo apt update
sudo apt install git curl build-essential python3-venv binutils-aarch64-linux-gnu
```

##### 2. Clone the project

```bash
git clone https://github.com/t3gemstone/ardupilot
cd ardupilot
git submodule update --init --recursive
./setup.sh
```

##### 3. Install Taskfile

```bash
curl --location https://github.com/go-task/task/releases/download/v3.49.1/task_3.49.1_linux_amd64.deb --output ~/task_3.49.1_linux_amd64.deb
sudo apt install ~/task_3.49.1_linux_amd64.deb
```

or

```bash
sh -c "$(curl --location https://taskfile.dev/install.sh)" -- -d -b /usr/local/bin
```

##### 4. Compile the project

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

[![SITL](.meta/youtube.png)](https://www.youtube.com/watch?v=wU4kDhm9MoE)

This repository includes a complete SITL environment for ArduPilot development and testing, orchestrated through Docker
Compose. The setup integrates ArduPilot SITL with QGroundControl and Gazebo simulation for realistic vehicle testing.

### Architecture

The SITL environment consists of three containerized services:

- **ArduPilot SITL**: Flight controller simulation running your vehicle firmware
- **QGroundControl**: Ground control station for mission planning and monitoring
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

### Connecting to SITL

QGroundControl automatically connects to **14550** UDP port.

### Configuration

SITL parameters are configured in the Taskfile:

```yaml
SITL_LOCATION: ARDUCOPTER_AUTOTEST
SITL_VEHICLE: ArduPlane
SITL_FRAME: gazebo-skywalker-x8-quad
SITL_WORLD: skywalker_x8_quad_runway.sdf
```

**SITL_LOCATION**: Starting location for the simulated vehicle
- Find available locations in [ArduPilot locations.txt](https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/locations.txt)
- Examples: `ARDUCOPTER_AUTOTEST`, `KSFO`, `CMAC`

**SITL_VEHICLE**: Vehicle type to simulate
- Examples: `ArduPlane`, `ArduCopter`, `ArduSub`, `Rover`

**SITL_FRAME**: Frame type for vehicle configuration
- Examples: `gazebo-skywalker-x8-quad`, `gazebo-iris`, `gazebo-zephyr`

**SITL_WORLD**: Gazebo world file for simulation environment
- Examples: `skywalker_x8_quad_runway.sdf`, `iris_runway.sdf`, `zephyr_runway.sdf`

After modifying any SITL variables in the Taskfile, restart the environment:

```bash
task sitl-start
```

The Docker Compose configuration will automatically pick up the new environment variables and restart the services with
the updated settings.

# AI Swarm

Gemstone ArduPilot project features an experimental AI-driven Drone Swarm simulation that lets you control a 4-drone ArduPlane (QuadPlane) swarm in Gazebo using natural language commands powered by a local Large Language Model (LLM).

[![Swarm](.meta/youtube2.png)](https://www.youtube.com/watch?v=slPr-Kdf_r0)

### Quick Start

1. **Start the Swarm Environment**:
   Begin by spinning up the simulation, including 4 ArduPilot SITL instances, Gazebo, QGroundControl, and a local instance of Ollama to host the LLM.
   ```bash
   newgrp docker
   task swarm-up
   ```

2. **Launch the AI CLI**:
   Once all containers are running and the LLM is pulled, start the Python-based CLI to issue natural language commands to your swarm.
   ```bash
   task swarm-cli
   ```

3. **Issue Natural Language Commands**:
   You can now type commands to the AI Swarm CLI. For example, to make the drones take off and form a V-formation, try:
   > "Let the lead drone take off to 100 meters. Let Drone 2, Drone 3 and Drone 4 take off and move to the left-back, right-back and full-back positions, respectively, and follow in the V formation."

   The autonomous AI calculates relative offsets and transmits coordinated MAVLink commands in parallel to coordinate complex swarm formations.

4. **Stop the Swarm Environment**:
   To stop the simulation and cleanly shut down the containers:
   ```bash
   task swarm-down
   ```

### Debugging & Logs

You can view the multi-container docker orchestration logs via:

```bash
task swarm-logs
```


### Available Models

| Model                                 | Image                                                                                                                       | SITL_VEHICLE | SITL_FRAME                  | SITL_WORLD                   |
|---------------------------------------|-----------------------------------------------------------------------------------------------------------------------------|--------------|-----------------------------|------------------------------|
| **BiCopter**                          | ![BiCopter](https://github.com/ArduPilot/SITL_Models/assets/24916364/0f41a75e-f356-4812-9407-9c19ec6f76a4)                  | ArduCopter   | gazebo-bicopter             | bicopter_runway.sdf          |
| **Hexapod Copter**                    | ![Hexapod Copter](https://user-images.githubusercontent.com/24916364/225340320-9aa31fe2-4602-4036-ba6b-491f72097c01.jpg)    | ArduCopter   | gazebo-hexapod-copter       | hexapod_copter_runway.sdf    |
| **Iris Copter**                       | ![Iris Copter](.meta/sitl-models/iris-copter.jpg)                                                                           | ArduCopter   | gazebo-iris                 | iris_runway.sdf              |
| **Alti Transition QuadPlane**         | ![Alti Transition](https://user-images.githubusercontent.com/24916364/150612555-958a64d4-c434-4f90-94bd-678e6b6011ec.png)   | ArduPlane    | gazebo-alti-transition-quad | alti_transition_runway.sdf   |
| **SkyCat TVBS QuadPlane**             | ![SkyCat TVBS](https://user-images.githubusercontent.com/24916364/145025150-4e7e48e1-3e83-4c83-be7b-b944db1d9152.png)       | ArduPlane    | gazebo-skycat-tvbs          | skycat_runway.sdf            |
| **Skywalker X8 Plane**                | ![Skywalker X8](.meta/sitl-models/skywalker-x8-plane.png)      | ArduPlane    | gazebo-skywalker-x8         | skywalker_x8_runway.sdf      |
| **Skywalker X8 QuadPlane**            | ![Skywalker X8 Quad](https://user-images.githubusercontent.com/24916364/142733947-1a39e963-0aea-4b1b-a57b-85455b2278fe.png) | ArduPlane    | gazebo-skywalker-x8-quad    | skywalker_x8_quad_runway.sdf |
| **Swan-K1 Tailsitter Quadplane**      | ![Swan-K1](https://user-images.githubusercontent.com/24916364/210408630-01e5f56d-57ba-430e-b04d-62cb8d232527.png)           | ArduPlane    | gazebo-swan-k1-hwing        | swan_k1_hwing_runway.sdf     |
| **Weight-Shift Plane**                | ![Weight-Shift](https://github.com/NDevDrone/SITL_Models/assets/50757802/9aee8639-d1a1-4807-8118-03f4ccdcc9ff)              | ArduPlane    | gazebo-wsc-aircraft         | wsc_aircraft_runway.sdf      |
| **X-UAV Mini Talon V-Tail**           | ![Mini Talon V-Tail](https://github.com/user-attachments/assets/44c3e42c-cb3e-4ca9-ba7b-c3e165276917)                       | ArduPlane    | gazebo-mini-talon-vtail     | vtail_runway.sdf             |
| **Zephyr Plane**                      | ![Zephyr Plane](.meta/sitl-models/zephyr-plane.png)                                                                         | ArduPlane    | gazebo-zephyr               | zephyr_runway.sdf            |
| **AION R1 Skid-steer Rover**          | ![AION R1](https://github.com/ArduPilot/SITL_Models/assets/24916364/58f25501-5863-423c-a5bd-6c3cbf9612e3)                   | Rover        | gazebo-r1-rover             | r1_rover_runway.sdf          |
| **Blue Robotics BlueBoat**            | ![BlueBoat](https://github.com/ArduPilot/SITL_Models/assets/24916364/11213e94-9e58-45eb-8181-1cec6c64ee19)                  | Rover        | gazebo-blueboat             | waves.sdf                    |
| **Catamaran**                         | ![Catamaran](https://github.com/ArduPilot/SITL_Models/assets/24916364/f0a65fc0-f25b-43ea-8690-f7eb979ed455)                 | Rover        | gazebo-catamaran            | catamaran_waves.sdf          |
| **DAF XF 450 Tractor**                | ![DAF XF 450 Tractor](.meta/sitl-models/daf-xf-450-tractor.png)                       | Rover        | gazebo-daf-xf-450-tractor   | daf_truck_runway.sdf         |
| **Omni3 Mecanum Rover**               | ![Omni3 Rover](https://github.com/user-attachments/assets/53d5d0bf-e4e3-4f03-a50a-4004a9865309)                             | Rover        | gazebo-omni3rover           | omnirover_playpen.sdf        |
| **Omni4 Mecanum Rover**               | ![Omni4 Rover](https://github.com/user-attachments/assets/00775e2c-a651-4902-9493-c272687152c0)                             | Rover        | gazebo-omni4rover           | omnirover_playpen.sdf        |
| **Quadruped Rover**                   | ![Quadruped](https://user-images.githubusercontent.com/24916364/144449710-5bab34b4-dabf-410f-b276-d290ddbb54b2.gif)         | Rover        | gazebo-quadruped            | quadruped_runway.sdf         |
| **Sawppy Rover**                      | ![Sawppy Rover](https://user-images.githubusercontent.com/24916364/210653579-e635ffc2-2962-4221-83a8-9622915a4121.png)      | Rover        | gazebo-sawppy               | sawppy_playpen.sdf           |
| **Wild Thumper 6WD Skid-steer Rover** | ![Wild Thumper](https://user-images.githubusercontent.com/24916364/144286154-231ac9b3-e54b-489f-b35e-bc2adb4b1aa0.png)      | Rover        | gazebo-wild-thumper         | wildthumper_runway.sdf       |

Checkout [SITL_Models documentation](https://github.com/ArduPilot/SITL_Models/tree/master/Gazebo/docs) for more
information about Gazebo models.
