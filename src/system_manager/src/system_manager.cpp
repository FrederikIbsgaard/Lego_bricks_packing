#include <iostream>
#include "ros/ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "system_manager");
    ros::NodeHandle n;

    ROS_INFO("System manager started!");

    return 0;
}