.PHONY: create_udev_rules delete_udev_rules

all: create_udev_rules build run clean

build:
	docker build -t ros-noetic-zsh:latest .

run:
	xhost +local:root
	-docker run -it \
	    --privileged \
		--env="DISPLAY" \
		-v /dev/ttyUSB0:/dev/ttyUSB0 \
		-v /dev/video0:/dev/video0 \
		-v /dev/ttyPlate:/dev/ttyPlate \
		-v /dev/ttyRplidar_s2:/dev/ttyRplidar_s2 \
		-v /dev/plate:/dev/plate \
		-v /dev/rplidar:/dev/rplidar \
		-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
		-e XDG_RUNTIME_DIR=/tmp \
		-e QT_X11_NO_MITSHM=1 \
		--net=host \
		--name ros-noetic-zsh \
		-p 9090:9090 \
		-p 8888:8888 \
		--ulimit nofile=1024:524288 \
		--mount type=bind,source=$(shell pwd)/catkin_ws,target=/root/catkin_ws \
		ros-noetic-zsh:latest
	xhost -local:root

clean:
	docker container rm ros-noetic-zsh
	docker rmi ros-noetic-zsh:latest

attach:
	-docker exec -it ros-noetic-zsh /bin/zsh


setup-usb:
	@chmod +x ./scripts/usb_setup.sh
	@./scripts/usb_setup.sh

create_udev_rules:
	@echo "Remap the serial port(ttyUSBX) to custom name"
	@echo " "

	@echo "Rplidar usb connection as /dev/rplidar"
	@echo "Plate usb connection as /dev/plate"
	@echo "check these using the command : ls -l /dev|grep ttyUSB"
	@echo " "

	@echo "Start copy rule files in scripts, to /etc/udev/rules.d/"
	@sudo cp ./scripts/rplidar.rules /etc/udev/rules.d
	@sudo cp ./scripts/plate.rules /etc/udev/rules.d
	@echo " "

	@echo "Restarting udev"
	@sudo udevadm control --reload-rules
	@sudo udevadm trigger
	@echo " "

	@echo "Finish usb port setup"
	@echo " "

delete_udev_rules:
	@echo "Delete the rules"
	@sudo rm /etc/udev/rules.d/rplidar.rules
	@sudo rm /etc/udev/rules.d/plate.rules
	@echo " "

	@echo "Restarting udev"
	@sudo udevadm control --reload-rules
	@sudo udevadm trigger
	@echo " "

	@echo "Finish delete rules"
	@echo " "

