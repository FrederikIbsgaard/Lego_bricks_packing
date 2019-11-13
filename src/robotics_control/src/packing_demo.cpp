#include "ros/ros.h"
#include <std_msgs/String.h>
#include "robot_control/goto_config.h"

#include <random>

std::random_device rd;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "packing_demo");
    ros::NodeHandle n;

    std::random_device rd;

    int randThresh = rd.max()/4;

    ros::ServiceClient robotClient = n.serviceClient<robot_control::goto_config>("/go_to_config");
    robot_control::goto_config robotCommand;
    robotCommand.request.config_name = "noConfig";

    for(int i = 0; i < 14; i++)
    {
        //Small:
        robotCommand.request.config_name = "preGraspSmall";
        ROS_INFO_STREAM("Going to: " << robotCommand.request.config_name);
        if(!robotClient.call(robotCommand))
        {
            ROS_ERROR("Failed to command robot.");
        }
        ros::Duration(0.1).sleep();

        robotCommand.request.config_name = "graspSmall";
        ROS_INFO_STREAM("Going to: " << robotCommand.request.config_name);
        if(!robotClient.call(robotCommand))
        {
            ROS_ERROR("Failed to command robot.");
        }
        ros::Duration(0.1).sleep();

        robotCommand.request.config_name = "preGraspSmall";
        ROS_INFO_STREAM("Going to: " << robotCommand.request.config_name);
        if(!robotClient.call(robotCommand))
        {
            ROS_ERROR("Failed to command robot.");
        }
        ros::Duration(0.1).sleep();

        if(rd() < randThresh) //Discard:
        {
            robotCommand.request.config_name = "aboveDiscard";
            ROS_INFO_STREAM("Going to: " << robotCommand.request.config_name);
            if(!robotClient.call(robotCommand))
            {
                ROS_ERROR("Failed to command robot.");
            }
        }
        else //box A:
        {
            robotCommand.request.config_name = "aboveBoxA";
            ROS_INFO_STREAM("Going to: " << robotCommand.request.config_name);
            if(!robotClient.call(robotCommand))
            {
                ROS_ERROR("Failed to command robot.");
            }
        }
        ros::Duration(0.1).sleep();

        //Medium:
        robotCommand.request.config_name = "preGraspMedium";
        ROS_INFO_STREAM("Going to: " << robotCommand.request.config_name);
        if(!robotClient.call(robotCommand))
        {
            ROS_ERROR("Failed to command robot.");
        }
        ros::Duration(0.1).sleep();

        robotCommand.request.config_name = "graspMedium";
        ROS_INFO_STREAM("Going to: " << robotCommand.request.config_name);
        if(!robotClient.call(robotCommand))
        {
            ROS_ERROR("Failed to command robot.");
        }
        ros::Duration(0.1).sleep();

        robotCommand.request.config_name = "preGraspMedium";
        ROS_INFO_STREAM("Going to: " << robotCommand.request.config_name);
        if(!robotClient.call(robotCommand))
        {
            ROS_ERROR("Failed to command robot.");
        }
        ros::Duration(0.1).sleep();

        if(rd() < randThresh) //Discard:
        {
            robotCommand.request.config_name = "aboveDiscard";
            ROS_INFO_STREAM("Going to: " << robotCommand.request.config_name);
            if(!robotClient.call(robotCommand))
            {
                ROS_ERROR("Failed to command robot.");
            }
        }
        else //box A:
        {
            robotCommand.request.config_name = "aboveBoxA";
            ROS_INFO_STREAM("Going to: " << robotCommand.request.config_name);
            if(!robotClient.call(robotCommand))
            {
                ROS_ERROR("Failed to command robot.");
            }
        }
        ros::Duration(0.1).sleep();

        //Large:
        robotCommand.request.config_name = "preGraspLarge";
        ROS_INFO_STREAM("Going to: " << robotCommand.request.config_name);
        if(!robotClient.call(robotCommand))
        {
            ROS_ERROR("Failed to command robot.");
        }
        ros::Duration(0.1).sleep();

        robotCommand.request.config_name = "graspLarge";
        ROS_INFO_STREAM("Going to: " << robotCommand.request.config_name);
        if(!robotClient.call(robotCommand))
        {
            ROS_ERROR("Failed to command robot.");
        }
        ros::Duration(0.1).sleep();

        robotCommand.request.config_name = "preGraspLarge";
        ROS_INFO_STREAM("Going to: " << robotCommand.request.config_name);
        if(!robotClient.call(robotCommand))
        {
            ROS_ERROR("Failed to command robot.");
        }
        ros::Duration(0.1).sleep();

        if(rd() < randThresh) //Discard:
        {
            robotCommand.request.config_name = "aboveDiscard";
            ROS_INFO_STREAM("Going to: " << robotCommand.request.config_name);
            if(!robotClient.call(robotCommand))
            {
                ROS_ERROR("Failed to command robot.");
            }
        }
        else //box A:
        {
            robotCommand.request.config_name = "aboveBoxA";
            ROS_INFO_STREAM("Going to: " << robotCommand.request.config_name);
            if(!robotClient.call(robotCommand))
            {
                ROS_ERROR("Failed to command robot.");
            }
        }
        ros::Duration(0.1).sleep();
    }

    return 0;
}