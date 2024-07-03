#!/bin/sh

# Create udev rule
echo 'SUBSYSTEM=="tty", ACTION=="add", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ttyRplidar_s2"' | sudo tee /etc/udev/rules.d/99-rplidar.rules
echo 'SUBSYSTEM=="tty", ACTION=="remove", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", RUN+="/bin/rm -f /dev/ttyRplidar_s2"' | sudo tee -a /etc/udev/rules.d/99-rplidar.rules

# Reload udev rules and trigger
sudo udevadm control --reload-rules
sudo udevadm trigger