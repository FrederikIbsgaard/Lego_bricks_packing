#include "ros/ros.h"
#include <ur_dashboard_msgs/RobotMode.h>
#include <ur_dashboard_msgs/SafetyMode.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>

ros::Publisher oeePub;
ros::Publisher packmlPub;
ros::Publisher lightBtnPub;

bool hasJustBeenSafeguardStopped = false;

bool hasJustBeenEstopped = false;

bool wasJustIdle = false;

bool wasJustPowerOff = false;

void safetymodeChanged(const ur_dashboard_msgs::SafetyMode::ConstPtr& msg);
void robotModeChanged(const ur_dashboard_msgs::RobotMode::ConstPtr& msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_state_monitor");
    ros::NodeHandle n;

    ros::Subscriber robotSafetyModeSub = n.subscribe<ur_dashboard_msgs::SafetyMode>("/ur_hardware_interface/safety_mode", 1, safetymodeChanged);
    ros::Subscriber robotModeSub = n.subscribe<ur_dashboard_msgs::RobotMode>("/ur_hardware_interface/robot_mode", 1, robotModeChanged);

    oeePub = n.advertise<std_msgs::String>("/oee_calculator", 10);
    packmlPub = n.advertise<std_msgs::Int8>("/action_state", 10);
    lightBtnPub = n.advertise<std_msgs::Int8>("/set_button_light", 10);

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
        

        if(wasJustPowerOff) //Means that e-stop btn has been released/twisted
        {
            //Turn on button light:
            std_msgs::Int8 i;
            i.data = 1;
            lightBtnPub.publish(i);
        }
        
        // if(hasJustBeenEstopped)
        // {
        //     std_msgs::Int8 packmlAction;
        //     packmlAction.data = 11; //AC_CLEAR
        //     packmlPub.publish(packmlAction);

        //     packmlAction.data = 0; //AC_SC
        //     packmlPub.publish(packmlAction);
        // }
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
        ROS_INFO_STREAM("HURRA1");
    }
}

void robotModeChanged(const ur_dashboard_msgs::RobotMode::ConstPtr& msg)
{
    ROS_INFO_STREAM("New robot mode: " << msg->mode);

    if(msg->mode == msg->RUNNING)
    {
        ROS_INFO_STREAM("Running");
        if(wasJustIdle)
        {
            std_msgs::Int8 i;
            i.data = 0; //AC_SC
            packmlPub.publish(i);
            wasJustIdle = false;
            ROS_INFO_STREAM("HURRA2");
        }
    }
    else if(msg->mode == msg->IDLE)
    {
        ROS_INFO_STREAM("IDLE");
        wasJustIdle = true;
    }
    else if(msg->mode == msg->BOOTING)
    {
        std_msgs::Int8 i;
        i.data = 11; //AC_CLEAR
        packmlPub.publish(i);
        ROS_INFO("Clearing!");

        //Turn off button light:
        std_msgs::Int8 j;
        j.data = 0;
        lightBtnPub.publish(j);
        wasJustPowerOff = false;
    }
    else if(msg->mode == msg->POWER_OFF)
    {
        wasJustPowerOff = true;
    }
}