FROM ardupilot/ardupilot-dev-base

RUN apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    git \
    lsb-release \
    sudo \
    tzdata \
    && git config --global url."https://github.com/".insteadOf git://github.com/ \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/t3gemstone/ardupilot.git /app/ardupilot

WORKDIR /app/ardupilot

RUN git submodule update --init --recursive

RUN ./waf distclean \
    && ./waf configure --board sitl \
    && ./waf copter \
    && ./waf plane \
    && ./waf rover \
    && ./waf sub

RUN pip3 install --no-cache-dir MAVProxy pymavlink

WORKDIR /app/parameters

RUN git clone --depth=1 https://github.com/ArduPilot/SITL_Models.git /tmp/SITL_Models \
    && cp /tmp/SITL_Models/Gazebo/config/* . \
    && rm -rf /tmp/SITL_Models

ENV AP_INSTANCE="--instance 0"
ENV AP_SPEEDUP="--speedup 1"
ENV AP_MODEL="--model JSON"
ENV AP_FRAME=
ENV AP_LOCATION="--location ARDUCOPTER_AUTOTEST"
ENV AP_VEHICLE="--vehicle ArduPlane"
ENV AP_PARAM="--add-param-file skywalker_x8_quad.param"

ENTRYPOINT ["sh", "-c", "/app/ardupilot/Tools/autotest/sim_vehicle.py \
    --wipe-eeprom \
    --no-rebuild \
    --no-mavproxy \
    --sitl-instance-args --serial0=udpclient:127.0.0.1:14550 \
    ${AP_INSTANCE} \
    ${AP_SPEEDUP} \
    ${AP_VEHICLE} \
    ${AP_FRAME} \
    ${AP_LOCATION} \
    ${AP_MODEL} \
    ${AP_PARAM}"]
