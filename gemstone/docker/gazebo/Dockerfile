FROM osrf/ros:humble-desktop

RUN apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential \
    git \
    gnupg \
    lsb-release \
    python3-pip \
    sudo \
    tzdata \
    wget \
    && git config --global url."https://github.com/".insteadOf git://github.com/ \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        gstreamer1.0-gl \
        gstreamer1.0-libav \
        gstreamer1.0-plugins-bad \
        gz-garden \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer1.0-dev \
        libgz-rendering8-dev \
        libgz-sim7-dev \
        libgz-sim8-dev \
        libopencv-dev \
        rapidjson-dev \
        ros-${ROS_DISTRO}-ros-gz \
    && rm -rf /var/lib/apt/lists/*

ARG UID=1000
ARG GID=1000
ARG USER=ros
ARG PWDR=/
RUN addgroup --gid ${GID} ${USER} \
    && adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} ${USER} \
    && usermod -a -G dialout ${USER} \
    && echo ${USER}" ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/99_aptget \
    && chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

ENV USER=${USER}
USER ${USER}

WORKDIR /home/${USER}

ENV GZ_VERSION=garden

RUN git clone --depth=1 https://github.com/ArduPilot/ardupilot_gazebo \
    && cd ardupilot_gazebo \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && cmake --build build

RUN git clone --depth=1 https://github.com/ArduPilot/SITL_Models.git

ENV GZ_SIM_RESOURCE_PATH=/home/${USER}/ardupilot_gazebo/models:\
/home/${USER}/ardupilot_gazebo/worlds:\
/home/${USER}/SITL_Models/Gazebo/models:\
/home/${USER}/SITL_Models/Gazebo/worlds
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/home/${USER}/ardupilot_gazebo/build

ENV GZ_WORLD=skywalker_x8_quad_runway.sdf

ENTRYPOINT [ "sh", "-c", "gz sim -v4 -r ${GZ_WORLD}" ]
