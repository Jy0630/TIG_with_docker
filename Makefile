all: build run clean
build:
	docker build -t ros-noetic-zsh:latest .
run:
	xhost +local:root
	-docker run -it \
	    --privileged \
		--env="DISPLAY" \
		-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
		-e XDG_RUNTIME_DIR=/tmp \
		-e QT_X11_NO_MITSHM=1 \
		--net=host \
		--name ros-noetic-zsh \
		--ulimit nofile=1024:524288 \
		--mount type=bind,source=$(shell pwd)/catkin_ws,target=/root/catkin_ws \
		ros-noetic-zsh:latest
	xhost -local:root
clean:
	docker container rm ros-noetic-zsh
	docker rmi ros-noetic-zsh:latest

attach:
	-docker exec -it ros-noetic-zsh /bin/zsh
