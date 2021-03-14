#!/bin/bash
echo  'SUBSYSTEM=="tty",DRIVERS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar.rules

echo  'SUBSYSTEM=="tty",DRIVERS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar-V2.rules

echo  'SUBSYSTEM=="tty",DRIVERS=="usb", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar-2303.rules

echo  'SUBSYSTEM=="tty",ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523",  SYMLINK+="arduino_nano"' >/etc/udev/rules.d/arduino.rules

sudo chmod 777 ~/../../dev/arduino_nano
sudo adduser $USER dialout

service udev reload
sleep 2
service udev restart
