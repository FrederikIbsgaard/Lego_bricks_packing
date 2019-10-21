
## ROS Configuration

* set ROS_HOSTNAME on robot PC to its own ip
* set ROS_HOSTNAME on base PC to its own ip
* set ROS_IP on robot PC to base IP
* set ROS_MASTER_URI to base IP:11311 both machines.


## RaspberryPi configuration

Prerequisites in the desktop system:

* Install network-manager
```
sudo apt-get install network-manager
```
* Install openssh
```
sudo apt install openssh-server
```
First we need to configure the static ip in the raspberry pi, to do so, acces the micro sd, and modify the file /etc/dhcpcd.conf adding the following lines:
```
interface eth0
static ip_address=192.168.0.4/24
static routers = 192.168.0.30
static domain_name_servers = 192.168.0.30 8.8.8.8
```
Follow this tutorial to configure the connection:

* [SSH](https://stackoverflow.com/a/39086537) - Raspberry pi ssh + ethernet connection configuration. 

In addition to the tutorial, the following step is required:
```
nm-connection-editor 
```
Select the connection created before. In ipv4/address:
```
   Address         Netmask      Gateway
192.168.0.30    255.255.255.0   0.0.0.0
```

Now the pi should be accessible. to access the pi via ssh:

```
ssh pi@raspberrypi.local
```
Insert the password = raspberry. 


# Camera module setup
* [PiCam](https://projects.raspberrypi.org/en/projects/getting-started-with-picamera?fbclid=IwAR1ityhSsclhhGkP4VA6zMV24nm3T6FkIS2gsmrrczlLi4mvsSZf7oAyw-g) - Getting started with the Camera Module. 
* [PiCam](https://desertbot.io/blog/how-to-stream-the-picamera?fbclid=IwAR25O8zClL-5L6vOdb-068ZDtUMsElLWPI1p4Co3r6ZezWPHVoHe9CDb7ho) - How to Stream the PiCamera to your Browser. 


# Configure the VPN 
* [VPN](https://www.instructables.com/id/Raspberry-Pi-SSH/) - Configure VPN and SSH

Acces the pi via ssh:
```
ssh pi@raspberrypi.local
```
run:
```
sudo raspi-config
```
Then choose advanced options >> SSH >> Enable SSH >> return to the Terminal and type:
```
sudo reboot
```
Once restarted the pi, the next step is installing the tightvncserver:
```
sudo apt-get install tightvncserver
```
Then start the VNC server (you'll be asked to set a password):
```
vncserver :1 -name RasPi -depth 16 -geometry 1024x768
```
It will ask for a password multiple times and it will set the server running.

In the laptop we need to install a VNC client:
```
sudo apt-get install vinagre
```
Run the client:
```
vinagre
```
Click:
```
|--> Connect
      |--> Choose a remote desktop to connect to:
            |--> Protocol
                  |--> VNC
```
This will update the tab:
```
|     |--> Choose a remote desktop to connect to:
|           |--> Host
|                 |--> 192.168.0.4:1  
```
Click the connect button. A new window will appear requesting the password previously created in order to access the server. Once introduced, the raspberry destok shoul be visible.
 
 # Installing ROS in the Pi
 As explained in the following tutorial:
 * [ROS](https://www.instructables.com/id/ROS-Melodic-on-Raspberry-Pi-4-RPLIDAR/) -ROS Melodic on Raspberry Pi 4[Debian Buster]

Toubleshootting:
Advisable to increase the swap size:
 * [pi](https://wpitchoune.net/tricks/raspberry_pi3_increase_swap_size.html) - Raspberry PI - increase swap size
Not able to resolve links to libboost:
 * [ROS](https://answers.ros.org/question/327497/compiling-ros-on-raspberry-pi-4-with-buster-problem-with-libboost158/)- Compiling ROS on raspberry pi 4 with Buster, problem with libboost1.58
 
```
sudo apt remove libboost1.67-dev
sudo apt autoremove
sudo apt install libboost1.58-dev libboost1.58-all-dev
sudo apt install g++-5 gcc-5
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 10
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 20
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 10
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 20
sudo update-alternatives --install /usr/bin/cc cc /usr/bin/gcc 30
sudo update-alternatives --set cc /usr/bin/gcc
sudo update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++ 30
sudo update-alternatives --set c++ /usr/bin/g++
sudo rm -rf ~/ros_catkin_ws/build_isolated
sudo rm -rf ~/ros_catkin_ws/devel_isolated
```
Couldn't resolve Opencv3 package installation:
 * [ROS](https://github.com/opencv/opencv/issues/14856#issuecomment-504416696) - error: invalid conversion from ‘const char*’ to ‘char*
 
Couldn't resolve ros_gui package:
 * [pi](https://raspberrypi.stackexchange.com/questions/62939/pyqt5-on-a-raspberry-pi) - PyQt5 on a Raspberry Pi
```
sudo apt-get update
sudo apt-get install qt5-default pyqt5-dev pyqt5-dev-tools
```
Couldn't resolve nodelet package dute to nullptr. We need to make sure that the package is compiled with 
 * [ROS](https://answers.ros.org/question/67784/c11-compatibility/) - C++11 compatibility
 
 To the CMake files:
```
gedit /home/pi/ros_catkin_ws/src/nodelet_core/nodelet/CMakeLists.txt
gedit /home/pi/ros_catkin_ws/src/common_tutorials/nodelet_tutorial_math/CMakeLists.txt
gedit /home/pi/ros_catkin_ws/src/common_tutorials/pluginlib_tutorials/CMakeLists.txt
gedit /home/pi/ros_catkin_ws/src/diagnostics/diagnostic_aggregator/CMakeLists.txt
gedit /home/pi/ros_catkin_ws/src/filters/CMakeLists.txt
gedit /home/pi/ros_catkin_ws/build_isolated/rosbag_storage/CMakeLists.txt
```
Add the following line:
```
# Line added to make sure that the compiler is correct:
if(CMAKE_COMPILER_IS_GNUCXX) 
  add_definitions(-std=gnu++0x) 
endif()
```
* [ROS](https://answers.ros.org/question/39657/importerror-no-module-named-rospkg/) - ImportError: No module named rospkg
```
 sudo apt-get purge python-rospkg && sudo apt-get update && sudo apt-get install -y ipython python-rospkg
```

 # Configuring the Pi

For the image processing the opencv library for python is required. Install it with the following comand:
```
pip3 install opencv-python
```

# Installing roslibpy
Following the guide at the repository:
* [roslibpy](https://github.com/gramaziokohler/roslibpy) - roslibpy: ROS Bridge library

```
pip3 install service_identity
```
Example can be found at
* [roslibpy](https://roslibpy.readthedocs.io/en/latest/examples.html#first-connection) - roslibpy: Examples
