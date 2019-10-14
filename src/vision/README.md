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



Configure the VPN [https://www.instructables.com/id/Raspberry-Pi-SSH/]:
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
 
Enable
Configuring the Pi

```
pip3 install opencv-python
```
