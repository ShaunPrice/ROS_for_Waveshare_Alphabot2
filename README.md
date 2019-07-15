# ROS for Waveshare Alphabot2
<p style="text-align:center"><img alighn="center" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/a/l/alphabot2-pi-3.jpg" alt="Waveshare Alphabot2"></p>
This repositiory is a ROS ([Robot Operating System](http://www.ros.org)) implementation for the [Waveshare Alphabot2 Pi](https://www.waveshare.com/product/robotics/alphabot2-pi-acce-pack.htm). IT's based on the  [Ubiquity Robotics](https://ubiquityrobotics.com/) [ROS for Raspberry Pi image](https://downloads.ubiquityrobotics.com/pi.html).

## TODO
- [ ] Fiducial rviz needs to be updated to display correctly.
- [ ] Fix issue with the IR Remote not receiving commands.
- [ ] Write a server and node for the ws2812b RGB LED's.
- [ ] Write demo applications.

## Getting Started with the Waveshare Alphabot2 and ROS
To run the ROS package you need to perform the following:

1. [Install the Raspberry Pi Image with ROS](#install-the-raspberry-pi-image-with-ros)
2. [Install the RGB LED library](#install-the-rgb-led-library)
3. [Configure the Camera library](#configure-the-camera-node)
4. [Install the Servo library](#install-the-servo-library)
5. [Copy over the Waveshare Alphabot2 ROS package code](#copy-over-the-waveshare-alphabot2-ros-package-code)
6. [Compile the ROS package](#compile-the-ros-package)
7. [Run the sample code to test it works](#run-the-sample-code-to-test-it-works)

## Install the Raspberry Pi Image with ROS
Install the Ubiquity Robotics ROS for Raspberry Pi image onto the Raspberry Pi controlling you Waveshare Alphabot2 Pi robot.

Download the image from here:

[ROS for Raspberry Pi image](https://downloads.ubiquityrobotics.com/pi.html)

Install the image onto your Raspberry Pi using the Ubiquity Robotics instructions.

## Install the RGB LED library
> This funtion is not currently finished. A server is required due to the permissions required.
Install the ws2812b RGB LED driver python library for the Raspberry PI:

    sudo pip install rpi_ws281x

The Ubiquity Robotic Raspberry Pi image includes a dtoverlay which configures the GPIO for some functions they have implemented. Unfortunately, this feature also turns on the Alphabot2's motors at startup. You need to remove this overlay from the **/boot/config.txt** file as follows:

    sudo nano /boot/config.txt

Go to the end of the file and remove or comment out (place a # in front of it) the following line:

    dtoverlay=ubiquity-led-buttons

Save the file (CTRL+x, y, Enter).

## Configure the Camera Node
The solution uses the UbiquityRobotics [raspicam_node](https://github.com/UbiquityRobotics/raspicam_node) which is installed on the Ubiquity Robotics ROS image by default. Use the instructions from the nodes [Github repository](https://github.com/UbiquityRobotics/raspicam_node) to configure it.

You will need the yaml files from the nodes raspicam_node-kinetic\raspicam_node-kinetic\camera_info directory in the following directory:

    /home/ubuntu/.ros/camera_info/

Set the **camera_info_url** parameter in the launch file to the appropriate yaml file. The camera that comes with the Waveshare Alphabot2 is a version 2 camera so you should use either of the folowing three configurations:

    camerav2_410x308.yaml
    camerav2_1280x720.yaml
    camerav2_1280x960.yaml

If you installed a version 1 camera you can use the following configureation:

    camerav1_1280x720.yaml

You should also add the following line to the end of the **/etc/modules** file to enable the vfl2 (Video for Linux 2) libraries:

    bcm2835-v4l2

If this is a neew image you can do this with:

    echo bcm2835-v4l2 | sudo tee -a /etc/modules

## Install the Servo library
Install the python libraries to enable communication with the PCA9685 servo using:

    sudo pip install smbus

## Copy over the Waveshare Alphabot2 ROS package code
Copy the **catkin_ws** directory and its content to the home folder of the Raspberry Pi. There should already be a **catkin_ws** folder there and you will be copying the code over the top of this folder. 

## Compile the ROS package
Change the working directory to **catkin_ws**:

    cd ~/catkin_ws

Compile and install the Waveshare_Alphabot2 package:

    catkin_make
    catkin_make install

## Run the sample code to test it works
Source the ROS code:

    source ~/catkin_ws/install/setup.bash

This is in the ~/bashrc file but we need to reload it after a build unless we reboot the robot.

Launch the Alphabot2 robot with the following launch file to make sure the servos and motors are working:

    roslaunch waveshare_alphabot2 Alphabot2_test.launch
