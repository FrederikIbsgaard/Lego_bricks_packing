
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
 

 # Configuring the Pi

For the image processing the opencv library for python is required. Install it with the following comand:
```
pip3 install opencv-python
```
