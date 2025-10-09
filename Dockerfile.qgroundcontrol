FROM ubuntu:22.04

RUN apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ca-certificates \
    fuse3 \
    gstreamer1.0-gl \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-bad \
    libfuse2 \
    libgl1-mesa-glx \
    libopengl0 \
    libxcb-cursor-dev \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    lsb-release \
    sudo \
    tzdata \
    && rm -rf /var/lib/apt/lists/*

ARG UID=1000
ARG GID=1000
ARG USER=qgc
ARG PWDR=/
RUN addgroup --gid ${GID} ${USER} \
    && adduser --gecos "QGC User" --disabled-password --uid ${UID} --gid ${GID} ${USER} \
    && usermod -a -G dialout ${USER} \
    && echo ${USER}" ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/99_aptget \
    && chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

ENV USER=${USER}
USER ${USER}

ADD --chown=${UID}:${GID} --chmod=755 https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage /usr/local/bin/QGroundControl

WORKDIR /home/${USER}

ENTRYPOINT [ "QGroundControl" ]
