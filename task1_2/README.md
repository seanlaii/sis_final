### Task 1 for using ML approach

## How to run

gdown --id 1THE1UHXECQRe1kaK6EC9DXwPnS6qmbr3

source docker_run.sh 

python task1_2.py

## Run with docker 

tx2 $  docker run -it --rm --name sis --device=/dev/nvhost-ctrl --device=/dev/nvhost-ctrl-gpu --device=/dev/nvhost-prof-gpu --device=/dev/nvmap --device=/dev/nvhost-gpu --device=/dev/nvhost-as-gpu -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra --net host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix --privileged -v /dev/bus/usb:/dev/bus/usb -v /home/$USER:/hosthome sis_competition

tx2 $  docker exec -it sis bash

docker $  cd catkin_ws/ && source devel/setup.sh
docker $  rosservice call /prediction
