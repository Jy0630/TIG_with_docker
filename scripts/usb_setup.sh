#!/bin/sh

# Lidar setup
echo 'SUBSYSTEM=="tty", ACTION=="add", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ttyRplidar_s2"' | sudo tee /etc/udev/rules.d/99-rplidar.rules
echo 'SUBSYSTEM=="tty", ACTION=="remove", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", RUN+="/bin/rm -f /dev/ttyRplidar_s2"' | sudo tee -a /etc/udev/rules.d/99-rplidar.rules

# Plate setup
echo 'SUBSYSTEM=="tty", ACTION=="add", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttyPlate"' | sudo tee /etc/udev/rules.d/99-plate.rules
echo 'SUBSYSTEM=="tty", ACTION=="remove", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", RUN+="/bin/rm -f /dev/ttyPlate"' | sudo tee -a /etc/udev/rules.d/99-plate.rules

# Reload udev rules and trigger
sudo udevadm control --reload-rules
sudo udevadm trigger