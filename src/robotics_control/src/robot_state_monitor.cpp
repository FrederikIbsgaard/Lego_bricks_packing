#include "ros/ros.h"
#include <ur_dashboard_msgs/RobotMode.h>
#include <ur_dashboard_msgs/SafetyMode.h>
#include <std_msgs/String.h>
// #include <std_msgs/Int8.h>

ros::Publisher oeePub;

bool hasJustBeenSafeguardStopped = false;

void safetymodeChanged(const ur_dashboard_msgs::SafetyMode::ConstPtr& msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_state_monitor");
    ros::NodeHandle n;

    ros::Subscriber robotSafetyModeSub = n.subscribe<ur_dashboard_msgs::SafetyMode>("/ur_hardware_interface/safety_mode", 1, safetymodeChanged);

    oeePub = n.advertise<std_msgs::String>("/oee_calculator", 10);

    ros::spin();
    return 0;
}

void safetymodeChanged(const ur_dashboard_msgs::SafetyMode::ConstPtr& msg)
{
    if(msg->mode == 5u)
    {
        ROS_INFO("SAFEGUARD STOP!");
        std_msgs::String s;
        s.data = "downTime_start";
        oeePub.publish(s);

        hasJustBeenSafeguardStopped = true;
    }
    else if(msg->mode == 1u || msg->mode == 2u) //Normal or reduced
    {
        if(hasJustBeenSafeguardStopped)
        {
            ROS_INFO("NORMAL");
            std_msgs::String s;
            s.data = "downTime_end";
            oeePub.publish(s);
            hasJustBeenSafeguardStopped = false;
        }
    }
    else if(msg->mode == 7u || msg->mode == 6u)
    {
        ROS_INFO("EMERGENCY STOP");
        std_msgs::String s;
        s.data = "STOP";
        oeePub.publish(s);
    }
}