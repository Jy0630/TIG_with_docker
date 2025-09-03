all: build run clean

build:
	docker build -t ros-noetic-zsh:latest .

run:
	xhost +local:root
	-docker run -it \
	    --privileged \
	    -e DISPLAY=$(DISPLAY) \
	    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
	    -v /home/itron/.docker.xauth:/root/.docker.xauth:rw \
	    -e XAUTHORITY=/root/.docker.xauth \
	    -e XDG_RUNTIME_DIR=/tmp \
	    -e QT_X11_NO_MITSHM=1 \
	    --net=host \
	    --name ros-noetic-zsh \
	    --ulimit nofile=1024:524288 \
	    --mount type=bind,source=$(shell pwd)/catkin_ws,target=/root/catkin_ws \
	    ros-noetic-zsh:latest
	xhost -local:root


clean:
	# Stop the container before removing it to prevent errors
	-docker stop ros-noetic-zsh
	-docker rm ros-noetic-zsh
	-docker rmi ros-noetic-zsh:latest

attach:
	-docker exec -it ros-noetic-zsh /bin/zsh
