#!/bin/zsh
source /opt/ros/noetic/setup.zsh && \
    source /root/catkin_ws/devel/setup.zsh && \
    cd /root/catkin_ws && \
    catkin_make

exec "$@"