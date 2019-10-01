# RSD 2019
## Discovered dependencies:
* [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu#Installation) - Ubuntu install of ROS Melodic
* [ROS](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) - Installing and Configuring Your ROS Environment
    
```
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```
  
## MES communication set up:   
Installing MySQL:
```
sudo apt install mysql-server -y
```

Checking installation
```
sudo systemctl status mysql.service
```
To see the current databases:
```
sudo mysqlshow -u root -p 
```

Accessing mysql prompt:
```
sudo mysql -u root -p
```
In the prompt, create database and user:

```
CREATE DATABASE rsd2018
CREATE USER 'rsd'@'localhost';
GRANT ALL PRIVILEGES ON rsd2018.* TO 'rsd'@'localhost' IDENTIFIED BY 'rsd2018';
```
