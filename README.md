sudo apt-get install ros-kinetic-ar-track-alvar
sudo apt-get install ros-kinetic-usb-cam
sudo apt-get install ros-indigo-compressed-image-transport
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/my_camera/image camera:=/my_camera
