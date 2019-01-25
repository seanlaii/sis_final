#!/usr/bin/env sh

# Setup the style of color
RED='\033[0;31m'
NC='\033[0m'

# Find current directory and transfer it to container directory for Docker
current_dir="$(pwd)"
host_dir="/home/$USER/"
container_dir="/hosthome/"
goal_dir=${current_dir//$host_dir/$container_dir}
echo ${goal_dir}

# Check the command 'nvidia-docker' is existing or not
ret_code="$(command -v nvidia-docker)"
if [ -z "$ret_code" ]
then
    printf "${RED}\"nvidia-docker\" is not found, so substitute docker. $NC\n"
    docker run -it --rm --name sis --device=/dev/nvhost-ctrl --device=/dev/nvhost-ctrl-gpu --device=/dev/nvhost-prof-gpu --device=/dev/nvmap --device=/dev/nvhost-gpu --device=/dev/nvhost-as-gpu -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra --net host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix --privileged -v /dev/bus/usb:/dev/bus/usb -v /home/$USER:/hosthome -w ${goal_dir} argnctu/sis_base_image:v4 bash
else
    printf "Run \"nvidia-docker\"\n"
    docker run -it --rm --device=/dev/nvhost-ctrl --device=/dev/nvhost-ctrl-gpu --device=/dev/nvhost-prof-gpu --device=/dev/nvmap --device=/dev/nvhost-gpu --device=/dev/nvhost-as-gpu -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra --net host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix --privileged -v /dev/bus/usb:/dev/bus/usb -v /home/$USER:/hosthome -w ${goal_dir} argnctu/sis_base_image:v4 bash
fi
