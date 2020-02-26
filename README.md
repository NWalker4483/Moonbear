sudo apt-get install ros-kinetic-ar-track-alvar
sudo apt-get install ros-kinetic-usb-cam
sudo apt-get install ros-kinetic-compressed-image-transport

sudo apt-get install ros-kinetic-openni-launch

sudo apt-get install ros-kinetic-rosserial-arduino
sudo apt-get install ros-kinetic-rosserial

sudo apt-get install ros-kinetic-octomap-server

sudo apt-get install g++-multilib || sudo apt-get --reinstall install libc6 libc6-dev

rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/my_camera/image camera:=/my_camera
