FROM ros:humble

SHELL ["/bin/bash", "-c"]

####### System full upgrade
RUN apt-get update && apt-get --with-new-pkgs upgrade -y

####### Essential packages
RUN apt-get update && apt-get install -y apt-utils
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Paris
RUN apt-get update && apt-get install -y --fix-missing \
    git vim curl build-essential zlib1g-dev libx11-dev libusb-1.0-0-dev freeglut3-dev liblapacke-dev \
    unzip libopenblas-dev libatlas-base-dev cmake make lsb-release tree \
    sudo ca-certificates gnupg-agent libssl-dev apt-transport-https \
    software-properties-common usbutils mesa-utils mesa-va-drivers vainfo \
    python3-pip python3-numpy libeigen3-dev libgflags-dev libdw-dev \
    libv4l-dev v4l-utils wget curl libnuma-dev libnuma1 libgles2-mesa python3-pykdl


# pip installs for python3
RUN pip install opencv-python-headless
RUN pip install ros2_numpy

####### Install Cyclone DDS
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

####### Install required ROS packages for camera topics
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-cv-bridge

###### Build ros2_ws
WORKDIR /ros2_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

###### Source ROS2
RUN echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
