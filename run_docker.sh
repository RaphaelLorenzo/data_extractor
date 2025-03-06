#!/bin/bash
IsRunning=`docker ps -f name=ros2_data_extractor | grep -c "ros2_data_extractor"`;
if [ $IsRunning -eq "0" ]; then
    xhost +local:docker
    docker run --rm \
        --name ros2_data_extractor \
        --gpus all \
        -e DISPLAY=$DISPLAY \
        -e XAUTHORITY=$XAUTHORITY \
        -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e 'QT_X11_NO_MITSHM=1' \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /tmp/docker_share:/tmp/docker_share \
        -v `pwd`:/ros2_ws/src/data_extractor \
        --net host \
        --ipc host \
        --pid host \
        --device /dev/dri \
        --device /dev/snd \
        --device /dev/input \
        --device /dev/bus/usb \
        --privileged \
        --ulimit rtprio=99 \
        --entrypoint /bin/bash \
        -ti ros2_data_extractor
else
    echo "Docker image is already running. Opening new terminal...";
    docker exec -ti ros2_data_extractor /bin/bash -c "cd /ros2_ws/src/data_extractor && /bin/bash"
fi
