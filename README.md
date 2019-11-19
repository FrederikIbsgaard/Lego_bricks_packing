# RSD 2019
## Discovered dependencies:
* [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu#Installation) - Ubuntu install of ROS Melodic
* [ROS](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) - Installing and Configuring Your ROS Environment
    
```bash
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install ros-melodic-rosbridge-server
sudo apt install ros-melodic-tf2-web-republisher
```
  
To make robot work, add the following below line 7, in universal_robot/ur5_e_moveit_config/launch/ur5_e_moveit_planning_execution.launch
    <remap unless="$(arg sim)" from="/follow_joint_trajectory" to="/scaled_pos_traj_controller/follow_joint_trajectory"/>
