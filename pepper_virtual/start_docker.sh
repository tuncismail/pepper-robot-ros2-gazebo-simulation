#!/usr/bin/env bash

# This scripts accepts a special first parameter --devel which mounts the current folder into the container so changes
# from inside or outside the container affect the repository

# This needs
# sudo apt-get install x11-xserver-utils
# and it's needed for the docker container to be able to use the host X
xhost +local:root

USE_NVIDIA=false


# Is there an nvidia card in the system?
if [[ $(lshw -C display 2> /dev/null | grep -i NVIDIA) ]]; then
    echo "Nvidia GPU in the system, trying to use it."
    # If we have nvidia-container-runtime...
    if nvidia-container-runtime -v > /dev/null; then
        # Check if we have an nvidia driver
        NVIDIA_DRIVER_VERSION=`nvidia-smi --query-gpu=driver_version --format=csv,noheader | awk '{ print substr($0, 0, 4) }'`
        if [ -z "$NVIDIA_DRIVER_VERSION" ]; then
            echo "No Nvidia driver version found, but nvidia-container-runtime is installed... can you run 'nvidia-smi?"
            echo "You may want to install a driver to take advantage of it."
        else
            USE_NVIDIA=true
        fi
    else
        echo "Warning:"
        echo "The system has a Nvidia GPU but doesn't have nvidia-container-runtime installed"
        echo "You may want to install it to take advantage of it, e.g.:"
        echo "sudo apt-get install nvidia-container-runtime"
        echo "You may need to add the nvidia repository, instructions here: https://nvidia.github.io/nvidia-container-runtime/"
    fi
    if ! "$USE_NVIDIA"; then
        echo "Using non-nvidia alternative."
    fi
fi


# If the user wants to develop on pepper_virtual (flag --devel) mount the folder of this script as it
if [ "$#" -ge "1" ]; then
	if [ "$1" == "--devel" ]; then
        # Remove this argument from "$@" so we can forward other arguments
        shift

        # Get script path
        # Absolute path to this script
        SCRIPT=$(readlink -f $0)
        # Absolute path this script is in
        SCRIPTPATH=`dirname $SCRIPT`
        REPOSITORY_PATH=`realpath $SCRIPTPATH`
        # Create the --volume command
        
        MOUNT_LOCAL_REPOSITORY_VOLUME="--volume $REPOSITORY_PATH:/catkin_ws/src/pepper_virtual:rw"

        echo
        echo "    *** DEVELOPER MODE ***"
        echo "    Your current repository folder ($REPOSITORY_PATH) is mounted in the docker image"
        echo "    This means you can develop editing the files directly on that path."
        echo "    It also means that the repository folder from inside of the docker image is overwritten by yours."
        echo
    fi
fi


XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth


# Compose the long list of .so files from Nvidia to make X work (yeah, this is absurd)
# This provides a list of all .so files installed from packages from 'nvidia-''
NVIDIA_SOS=$(dpkg -l | grep nvidia- | awk '{print $2}' | xargs dpkg-query -L | grep lib | grep .so)
# This generates --volume /path/to/.so:/path/to/.so lines for every .so
for so in $NVIDIA_SOS; do DOCKER_NVIDIA_SO_VOLUMES+="--volume $so:$so "; done

# ram, also QT_X11_NO_XRENDER may help with some glitches, so leaving it here in case 
# we want to use it (but it may consume more cpu/bring other glitches)

if "$USE_NVIDIA"; then
    # If executing this script from an unmanned shell (like from a piece of code that doesn't have a terminal starting it)
    # We can't use "-i"
    if [ -t 1 ]; then
        TERMINAL_FLAGS='-it'
    else
        TERMINAL_FLAGS='-t'
    fi
    docker run $TERMINAL_FLAGS \
        $DOCKER_NVIDIA_SO_VOLUMES \
        --volume $XSOCK:$XSOCK:rw \
        $MOUNT_LOCAL_REPOSITORY_VOLUME \
        --env DISPLAY=$DISPLAY \
        --env QT_X11_NO_XRENDER=0 \
        --env ROS_MASTER_URI=$ROS_MASTER_URI \
        --env ROS_IP \
        --net host \
        --pid host \
        --privileged \
        --name=pepper-virtual-container \
        frietz58/pepper-virtual:with-gazebo-files "$@"
        # Note: no --rm provided so the container won't disappear
fi
        # --env XAUTHORITY=$XAUTH \
        # --volume $XAUTH:$XAUTH:rw \

xhost -local:root
