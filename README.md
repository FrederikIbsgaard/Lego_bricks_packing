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
Accessing mysql prompt while importing the database structure (obtained from blackboard):
```
 sudo mysql -u root -p rsd2018 < ~/catkin_ws/src/Lego_bricks_packing/scripts/MES/db_export.sql
```
In the prompt, create user:
```
use rsd2018
CREATE DATABASE rsd2018
CREATE USER 'rsd'@'localhost';
GRANT ALL PRIVILEGES ON rsd2018.* TO 'rsd'@'localhost' IDENTIFIED BY 'rsd2018';
#source  ~/catkin_ws/src/Lego_bricks_packing/scripts/MES/db_export.sql
exit
```
Installing flask
```
pip3 install flask-mysql

```
Running the flask server:
```
python3 ~/catkin_ws/src/Lego_bricks_packing/scripts/MES/rsd_2019_app_public.py 
```
Running the requesting order script:
```
python3 ~/catkin_ws/src/Lego_bricks_packing/scripts/MES/ordering_client.py 
```



