all: build run clean
build:
	docker build -t ros-noetic-zsh:latest .
run:
	-docker run -it \
		--name ros-noetic-zsh \
		--ulimit nofile=1024:524288 \
		--mount type=bind,source=$(shell pwd)/catkin_ws,target=/root/catkin_ws \
		ros-noetic-zsh:latest
clean:
	docker container rm ros-noetic-zsh
	docker rmi ros-noetic-zsh:latest

attach:
	-docker exec -it ros-noetic-zsh /bin/zsh
