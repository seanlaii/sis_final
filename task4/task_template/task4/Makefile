demo-docker-no-visual:
	bash -c "docker run -it --rm --name sis --net host --privileged -v /dev/bus/usb:/dev/bus/usb sis_competition"
demo-docker-bash:
	bash -c "docker run -it --rm --name sis --net host --privileged -v /dev/bus/usb:/dev/bus/usb sis_competition bash"
demo-docker-with-visual:
	bash -c "docker run -it --rm --name sis --net host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix --privileged -v /dev/bus/usb:/dev/bus/usb sis_competition"
demo-docker-exec:
	bash -c "docker exec -it sis bash"
