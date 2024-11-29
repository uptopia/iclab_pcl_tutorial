FROM nvidia/cuda:11.1.1-cudnn8-devel-ubuntu20.04

############################## SYSTEM PARAMETERS ##############################
# * Arguments
ARG USER=initial
ARG GROUP=initial
ARG UID=1000
ARG GID="${UID}"
ARG SHELL=/bin/bash
ARG HARDWARE=x86_64
ARG ENTRYPOINT_FILE=entrypoint.sh

# * Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
# ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# * Setup users and groups
RUN groupadd --gid "${GID}" "${GROUP}" \
    && useradd --gid "${GID}" --uid "${UID}" -ms "${SHELL}" "${USER}" \
    && mkdir -p /etc/sudoers.d \
    && echo "${USER}:x:${UID}:${UID}:${USER},,,:/home/${USER}:${SHELL}" >> /etc/passwd \
    && echo "${USER}:x:${UID}:" >> /etc/group \
    && echo "${USER} ALL=(ALL) NOPASSWD: ALL" > "/etc/sudoers.d/${USER}" \
    && chmod 0440 "/etc/sudoers.d/${USER}"

# * Replace apt urls and Time zone
# ? Change to tku
ENV TZ=Asia/Taipei
RUN sed -i 's@archive.ubuntu.com@ftp.tku.edu.tw@g' /etc/apt/sources.list \
    && ln -snf /usr/share/zoneinfo/"${TZ}" /etc/localtime && echo "${TZ}" > /etc/timezone
# ? Change to Taiwan
# RUN sed -i 's@archive.ubuntu.com@tw.archive.ubuntu.com@g' /etc/apt/sources.list \
#     && ln -snf /usr/share/zoneinfo/"${TZ}" /etc/localtime && echo "${TZ}" > /etc/timezone

# * Copy custom configuration
# ? Requires docker version >= 17.09
COPY --chmod=0775 ./${ENTRYPOINT_FILE} /entrypoint.sh
COPY --chown="${USER}":"${GROUP}" --chmod=0775 config config

############################### INSTALL #######################################
# * Install packages
RUN apt update \
    && apt install -y --no-install-recommends \
        sudo \
        vim \
        git \
        htop \
        cmake \
        curl \
        # * Shell
        terminator \
        # * base tools
        python3-pip \
        python3-dev \
        python3-setuptools \
        # * ROS depend tools
        lsb-release \
        python3-opencv \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# RUN ./config/pip/pip_setup.sh

# * Install ROS noetic
ENV ROS_DISTRO noetic
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
    && apt update \
    && dpkg --configure -a \
    && apt install -y -f --fix-broken --no-install-recommends \
        ros-$ROS_DISTRO-desktop-full \
        # * ROS depend tools
        ros-$ROS_DISTRO-ddynamic-reconfigure \
        ros-$ROS_DISTRO-rgbd-launch \
        ros-$ROS_DISTRO-catkin \
        python3-catkin-tools \ 
        python3-rospkg \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >>/home/$USER/.bashrc

# * Install Intel® RealSense™ SDK (librealsense)
#https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
RUN mkdir -p /etc/apt/keyrings \
    && curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null \
    && echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
        tee /etc/apt/sources.list.d/librealsense.list \
    && apt update \
    && apt install -y --no-install-recommends \
        # * Install Realsense libraries
        librealsense2-utils \
        librealsense2-dkms \
        # # * Install Realsense support ROS1 (https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy)
        ros-$ROS_DISTRO-realsense2-camera \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /

############################## USER CONFIG ####################################
# * Switch user to ${USER}
USER ${USER}

# RUN ./config/shell/bash_setup.sh "${USER}" "${GROUP}" \
#     && ./config/shell/terminator/terminator_setup.sh "${USER}" "${GROUP}" \
#     && ./config/shell/tmux/tmux_setup.sh "${USER}" "${GROUP}" \
#     && sudo rm -rf /config

# * Install realsense-ros 
# https://blog.csdn.net/z1872385/article/details/130578113
# * Create realsense-ros folder
WORKDIR /home/"${USER}"/realsense_ros/

#----use Realsense D435i (ROS1)----#
RUN sudo git clone https://github.com/IntelRealSense/realsense-ros.git src \
    && sudo /bin/bash -c "cd /home/"${USER}"/realsense_ros/src/; git checkout ros1-legacy;" \
    && sudo /bin/bash -c "cd /home/"${USER}"/realsense_ros/;. /opt/ros/noetic/setup.bash; catkin_make;" \
    && sudo echo "source /home/"${USER}"/realsense_ros/devel/setup.bash" >> ~/.bashrc

# * Switch workspace to ~/work
WORKDIR /home/"${USER}"/work

# * Make SSH available
EXPOSE 22

# ENTRYPOINT [ "/entrypoint.sh", "terminator" ]
# ENTRYPOINT [ "/entrypoint.sh", "tmux" ]
ENTRYPOINT [ "/entrypoint.sh", "bash" ]
# ENTRYPOINT [ "/entrypoint.sh" ]
