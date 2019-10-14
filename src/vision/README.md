set ROS_HOSTNAME on robot PC to its own ip,

set ROS_HOSTNAME on base PC to its own ip

set ROS_IP on robot PC to base IP

set ROS_MASTER_URI to base IP:11311 both machines.


RaspberryPi configuration

Prerequisites

Install network-manager
```
sudo apt-get install network-manager
```
Install openssh

```
sudo apt install openssh-server
```

Follow this tutorial to configure the connection:

https://stackoverflow.com/a/39086537

In ipv4/address:
   Address         Netmask      Gateway
192.168.0.30    255.255.255.0   0.0.0.0

To access the pi via ssh:

```
ssh pi@raspberrypi.local
```
password = raspberry


Camera module setup
```
This is a headline
```   

https://projects.raspberrypi.org/en/projects/getting-started-with-picamera?fbclid=IwAR1ityhSsclhhGkP4VA6zMV24nm3T6FkIS2gsmrrczlLi4mvsSZf7oAyw-g

https://desertbot.io/blog/how-to-stream-the-picamera?fbclid=IwAR25O8zClL-5L6vOdb-068ZDtUMsElLWPI1p4Co3r6ZezWPHVoHe9CDb7ho
