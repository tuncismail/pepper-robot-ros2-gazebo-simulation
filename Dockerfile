# Pepper Robot Gazebo Simulation - ROS 2 Humble
FROM --platform=linux/amd64 ros:humble-ros-base-jammy

LABEL maintainer="Ismail Tunc <tuncismail@users.noreply.github.com>"
LABEL description="Pepper robot simulation with ROS 2 Humble + Gazebo Classic 11"

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get upgrade -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget \
    python3-pip python3-colcon-common-extensions python3-rosdep \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-sensor-msgs \
    ros-humble-sensor-msgs-py \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-std-msgs \
    ros-humble-message-filters \
    ros-humble-laser-geometry \
    ros-humble-pcl-ros \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-topic-tools \
    ros-humble-rviz2 \
    python3-opencv \
    python3-numpy \
    xvfb \
    tigervnc-standalone-server \
    tigervnc-common \
    novnc \
    websockify \
    openbox \
    x11-utils \
    mesa-utils \
    tini \
    netcat \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN mkdir -p /colcon_ws/src && chown -R $USERNAME:$USERNAME /colcon_ws

# ---------- Patch gazebo_ros2_control to fix URDF-in-argv crash ----------
# The apt package passes the entire robot_description URDF as a --param
# argv to rcl_parse_arguments(), which fails on large URDFs (like Pepper's).
# Fix: remove the argv path and set robot_description via NodeOptions instead.
COPY patches/fix_gazebo_ros2_control_urdf_argv.patch /tmp/
RUN cd /tmp && \
    git clone --depth 1 --branch 0.4.10 \
        https://github.com/ros-controls/gazebo_ros2_control.git && \
    cd gazebo_ros2_control && \
    git apply /tmp/fix_gazebo_ros2_control_urdf_argv.patch && \
    cp -r /tmp/gazebo_ros2_control /colcon_ws/src/gazebo_ros2_control

COPY --chown=$USERNAME:$USERNAME gazebo_model_velocity_plugin  /colcon_ws/src/gazebo_model_velocity_plugin
COPY --chown=$USERNAME:$USERNAME pepper_virtual                /colcon_ws/src/pepper_virtual
COPY --chown=$USERNAME:$USERNAME pepper_robot                  /colcon_ws/src/pepper_robot
COPY --chown=$USERNAME:$USERNAME pepper_meshes                 /colcon_ws/src/pepper_meshes
COPY --chown=$USERNAME:$USERNAME pepper_laser_bridge           /colcon_ws/src/pepper_laser_bridge
COPY --chown=$USERNAME:$USERNAME velocity_bridge               /colcon_ws/src/velocity_bridge
COPY --chown=$USERNAME:$USERNAME test_e2e_headless.sh          /colcon_ws/src/test_e2e_headless.sh

RUN find /colcon_ws/src -name "*.py" -exec chmod +x {} \; && \
    find /colcon_ws/src -name "*.sh" -exec chmod +x {} \;

USER $USERNAME
WORKDIR /colcon_ws
RUN /bin/bash -c \
    "source /opt/ros/humble/setup.bash && \
     colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1"

ENV GAZEBO_MODEL_PATH=/colcon_ws/src/pepper_virtual/pepper_gazebo_plugin/models:/colcon_ws/install/pepper_description/share:/colcon_ws/install/pepper_meshes/share
ENV GAZEBO_PLUGIN_PATH=/colcon_ws/install/gazebo_model_velocity_plugin/lib

RUN echo 'source /opt/ros/humble/setup.bash' >> /home/$USERNAME/.bashrc && \
    echo 'source /colcon_ws/install/setup.bash' >> /home/$USERNAME/.bashrc

RUN mkdir -p /home/$USERNAME/.vnc && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME/.vnc

EXPOSE 6080
EXPOSE 5901

COPY --chown=$USERNAME:$USERNAME docker-entrypoint.sh /home/$USERNAME/docker-entrypoint.sh
RUN chmod +x /home/$USERNAME/docker-entrypoint.sh

WORKDIR /colcon_ws
ENTRYPOINT ["/home/rosuser/docker-entrypoint.sh"]
CMD ["gui"]
