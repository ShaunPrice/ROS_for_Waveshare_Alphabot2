# ROS for Waveshare Alphabot2
<p style="text-align:center"><img alighn="center" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/a/l/alphabot2-pi-3.jpg" alt="Waveshare Alphabot2"></p>
This repositiory is a ROS ([Robot Operating System](http://www.ros.org)) implementation for the [Waveshare Alphabot2 Pi](https://www.waveshare.com/product/robotics/alphabot2-pi-acce-pack.htm). IT's based on the  [Ubiquity Robotics](https://ubiquityrobotics.com/) [ROS for Raspberry Pi image](https://downloads.ubiquityrobotics.com/pi.html).

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

The Ubiquity Robotic Raspberry Pi image includes a dtoverlay which configures the GPIO for some functions they have implemented. Unfortunately, this feature also turns on the Alphabot2's motors at startup. You need to remove this overlay from the **/boot/config.txt** file as follows:

    sudo nano /boot/config.txt

Go to the end of the file and remove or comment out (place a # in front of it) the following line:

    dtoverlay=ubiquity-led-buttons

Save the file (CTRL+x, y, Enter).

## Install the RGB LED library
This service is required for the ROS code to talk to communicate with the RGB LED's on the robot. The communication requires elevated priviledges which is obtained by running the following python flask web application on port 2812. The service is only visibly locally to other applications on the robot.

Install the dependancies:

    sudo pip install rpi_ws281x flask

Install the RGB LED service to enable ROS to control the RGB LED's:

    cd ~/wsRGB
    chmod +x setup.sh
    sudo ./setup.sh

To start and stop the service manually you can use the following commands:

    sudo systemctl stop wsrgb.service          #To stop running service 
    sudo systemctl start wsrgb.service         #To start running service 
    sudo systemctl restart wsrgb.service       #To restart running service 
    sudo systemctl status wsrgb.service        #To view the status of the service 

To test if the service is running you can try some of the following commands:

    curl -i -X PUT http://localhost:2812/setLED/ff0000/00ff00/0000ff/ff00ff/
    curl -i -X PUT http://localhost:2812/setMode/theaterChaseRainbow/50/
    curl -i -X PUT http://localhost:2812/setMode/colorWipe/ff00ff/50/
    curl -i -X PUT http://localhost:2812/setAllRed/  
    curl -i -X PUT http://localhost:2812/setAllGreen/  
    curl -i -X PUT http://localhost:2812/setAllBlue/
    curl -i -X PUT http://localhost:2812/setDelay/250/  
    curl -i -X PUT http://localhost:2812/setMode/theaterChase/ff00ff/100/

To test using ROS use the following commands once ROS is installed and running:

    rostopic pub /rgb_leds waveshare_alphabot2/RGB_LED "'setLED'" "''" "'ff0000'" "'00ff00'" "'0000ff'" "'ff00ff'" 50
    rostopic pub /rgb_leds waveshare_alphabot2/RGB_LED "'setAllRed'" "''" "''" "''" "''" "''" 50
    rostopic pub /rgb_leds waveshare_alphabot2/RGB_LED "'setAllGreen'" "''" "''" "''" "''" "''" 50
    rostopic pub /rgb_leds waveshare_alphabot2/RGB_LED "'setAllBlue'" "''" "''" "''" "''" "''" 50
    rostopic pub /rgb_leds waveshare_alphabot2/RGB_LED "'colorWipe'" "'ff00ff'" "''" "''" "''" "''" 50
    rostopic pub /rgb_leds waveshare_alphabot2/RGB_LED "'rainbow'" "''" "''" "''" "''" "''" 50
    rostopic pub /rgb_leds waveshare_alphabot2/RGB_LED "'theaterChaseRainbow'" "''" "''" "''" "''" "''" 50
    rostopic pub /rgb_leds waveshare_alphabot2/RGB_LED "'theaterChase'" "'ffffff'" "''" "''" "''" "''" 50

## Install the IR Remote
The installation uses the lirc remote package and assumes your Waveshare Alphabot 2 came with a Car MP3 remote. If not you will need to configure the remote using the **irrecord** utility.

**The following was updated from the following web page: http://ozzmaker.com/how-to-control-the-gpio-on-a-raspberry-pi-with-an-ir-remote/**

Install the lirc packages:

    sudo apt-get install lirc liblircclient-dev ir-keytable

Edit the **/etc/modules** file:

    sudo nano /etc/modules

Append the following lines:

    lirc_dev
    lirc_rpi gpio_in_pin=17

Edit the **/boot/config.txt** file:

    sudo nano /boot/config.txt

Uncomment the line starting with **#dtoverlay=lirc-rpi** and append the pin as follows: 

    dtoverlay=lirc-rpi,gpio_in_pin=17

Go to the ~/lirc directory and run the following:

    cd ~/lirc
    sudo cp hardware.conf /etc/lirc/hardware.conf

Reboot the rebot.

Log back in and test it works:

    sudo /etc/init.d/lirc stop
    mode2 -d /dev/lirc0

Press some buttons on the remote and you should see something simular to the following:

    pulse 627
    space 514
    pulse 624
    space 513
    pulse 599
    space 521
    pulse 618
    space 1668
    pulse 589
    space 532

Go to the ~/lirc directory and run the following:

    cd ~/lirc
    chmod +x setup.sh
    sudo ./setup.sh

To test the remote is working and the codes are converted type the following command:

    irw

You should receive something like the following:

    0000000000ff30cf 00 KEY_1 CAR_MP3
    0000000000ff30cf 01 KEY_1 CAR_MP3
    0000000000ff6897 00 KEY_0 CAR_MP3
    0000000000ff6897 01 KEY_0 CAR_MP3
    0000000000ff38c7 00 KEY_5 CAR_MP3
    0000000000ffa25d 00 KEY_CHANNELDOWN CAR_MP3
    0000000000ffa25d 01 KEY_CHANNELDOWN CAR_MP3
    0000000000ffe21d 00 KEY_CHANNELUP CAR_MP3
    0000000000ffe21d 01 KEY_CHANNELUP CAR_MP3
    0000000000ff22dd 00 KEY_PREVIOUS CAR_MP3
    0000000000ff02fd 00 KEY_NEXT CAR_MP3
    0000000000ff02fd 01 KEY_NEXT CAR_MP3
    0000000000ffe01f 00 KEY_DOWN CAR_MP3
    0000000000ffa857 00 KEY_UP CAR_MP3

Your IR Remote Control is complete.

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

## Make the Python nodes and scripts executable

    sudo chmod +x ~/catkin_ws/src/waveshare_alphabot2/nodes/*
    sudo chmod +x ~/catkin_ws/src/waveshare_alphabot2/scripts/*.*

**Note:** If you also add the Alphabot2_*.sh files to the home directory the **Alphabot2_fix.sh** script will do this for you. You'll have to make this script executable first with **chmod +x Alphabot2_fix.sh** and run it with sudo (**sudo ./Alphabot2_fix.sh**).

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

## Alphabot2 Shell scripts
There are serveral Alphabot2 shell scripts in the home directory. THese are used for building and running the Alphabot2 ROS packages. You'll need to make them executable with the command **chmod +x Alphabot2_*.sh** before using them.

The scripts have the following functions:

    Alphabot2_build.sh                          # Builds the Alphabot2 ROS packages
    Alphabot2_clean.sh                          # Cleans the ROS catkin environment. Usefull if your having issues. run **./Alphabot2_build.sh** to rebuild the ROS packages. 
    Alphabot2_fix.sh                            # Correct permissions on the ROD Python nodes and scripts. They must be executable for ROS to find them.
    Alphabot2_run_clear_map.sh                  # Starts ROS and clears the Ubiquity Roboltics Fiducial SLAM example map generated with SLAM.
    Alphabot2_run_fiducial_localization.sh      # Starts ROS with the Ubiquity Roboltics Fiducial SLAM example in localisation only mode.
    Alphabot2_run_fiducial_rviz.sh              # Starts ROS rviz for visualisation when running the Ubiquity Roboltics Fiducial SLAM example.
    Alphabot2_run_fiducial_slam.sh              # Starts ROS with the Ubiquity Roboltics Fiducial SLAM example.
    Alphabot2_run.sh                            # Starts ROS with the base configuration for the Alphabot2 robot.
    Alphabot2_run_test.sh                       # Starts ROS with the base configuration for the Alphabot2 robot and tests the basic functionality.
