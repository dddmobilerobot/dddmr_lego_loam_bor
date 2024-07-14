#!/bin/bash

xhost +local:docker

is_x64=$(docker image ls dddmr_gtsam | grep x64)
is_cuda=$(docker image ls dddmr_gtsam | grep 12.4.1)
is_l4t_r36=$(docker image ls dddmr_gtsam | grep l4t_r36)
if [ "$is_cuda" != "" ] ;then 
    docker run -it \
        --privileged \
        --network=host \
        --runtime=nvidia\
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="NVIDIA_VISIBLE_DEVICES=all"\
        --env="NVIDIA_DRIVER_CAPABILITIES=all"\
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="${HOME}/dddmr_bags:/root/dddmr_bags" \
        --volume="${HOME}/dddmr_navigation:/root/dddmr_navigation" \
        --name="dddmr_ros2_dev" \
        dddmr_gtsam:12.4.1-cudnn-devel-ubuntu22.04
elif [ "$is_x64" != "" ] ;then 
    docker run -it \
        --privileged \
        --network=host \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="${HOME}/dddmr_bags:/root/dddmr_bags" \
        --volume="${HOME}/dddmr_navigation:/root/dddmr_navigation" \
        --name="dddmr_ros2_dev" \
        dddmr_gtsam:x64
elif [ "$is_l4t_r36" != "" ] ;then 
    docker run -it \
        --privileged \
        --network=host \
        --runtime=nvidia\
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="NVIDIA_VISIBLE_DEVICES=all"\
        --env="NVIDIA_DRIVER_CAPABILITIES=all"\
        --volume="/usr/bin/tegrastats:/usr/bin/tegrastats" \
        --volume="/usr/local/cuda-11.4/targets/aarch64-linux:/usr/local/cuda-11.4/targets/aarch64-linux" \
        --volume="/usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra" \
        --volume="/lib/modules:/lib/modules" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="${HOME}/dddmr_bags:/root/dddmr_bags" \
        --volume="${HOME}/dddmr_navigation:/root/dddmr_navigation" \
        --name="dddmr_ros2_dev" \
        dddmr_gtsam:l4t_r36.2
fi


