#include "ros/ros.h"
#include <ur_dashboard_msgs/RobotMode.h>
#include <ur_dashboard_msgs/SafetyMode.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>

ros::Publisher oeePub;
ros::Publisher packmlPub;

bool hasJustBeenSafeguardStopped = false;

bool hasJustBeenEstopped = false;

void safetymodeChanged(const ur_dashboard_msgs::SafetyMode::ConstPtr& msg);
void robotModeChanged(const ur_dashboard_msgs::RobotMode::ConstPtr& msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_state_monitor");
    ros::NodeHandle n;

    ros::Subscriber robotSafetyModeSub = n.subscribe<ur_dashboard_msgs::SafetyMode>("/ur_hardware_interface/safety_mode", 1, safetymodeChanged);

    oeePub = n.advertise<std_msgs::String>("/oee_calculator", 10);
    packmlPub = n.advertise<std_msgs::Int8>("/action_state", 10);

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
        
        if(hasJustBeenEstopped)
        {
            std_msgs::Int8 packmlAction;
            packmlAction.data = 11; //AC_CLEAR
            packmlPub.publish(packmlAction);

            packmlAction.data = 0; //AC_SC
            packmlPub.publish(packmlAction);
        }
    }
    else if(msg->mode == 7u || msg->mode == 6u) //E-stop (robot/system)
    {
        ROS_INFO("EMERGENCY STOP");
        hasJustBeenEstopped = true;
        std_msgs::String s;
        s.data = "STOP";
        oeePub.publish(s);

        std_msgs::Int8 packmlAction;
        packmlAction.data = 10; //AC_ABORT
        packmlPub.publish(packmlAction);

        packmlAction.data = 0; //AC_SC
        packmlPub.publish(packmlAction);
    }
}

void robotModeChanged(const ur_dashboard_msgs::RobotMode::ConstPtr& msg)
{
    if(msg->mode == msg->RUNNING)
    {
        std_msgs::Int8 i;
        i.data = 0;
        packmlPub.publish(i);
        ROS_INFO("State complete!");
    }
}