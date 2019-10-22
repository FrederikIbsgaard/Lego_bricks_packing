#include <iostream>
#include "ros/ros.h"
 #include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "lego_bricks_packing/goto_config.h"
#include <std_msgs/String.h>

#include <mutex>

//Configs:
std::vector<double> pickupSmall = {1.17439, -0.663472, 1.41558, -2.69422, -1.61225, 0.0158134};
std::vector<double> pickupMedium = {1.03682, -0.663927, 1.41798, -2.64278, -1.51245, -0.164062};
std::vector<double> pickupLarge = {0.894718, -0.697766, 1.41983, -2.63806, -1.48795, -0.164074};

std::vector<double> box = {1.00412, -1.14127, 1.99322, -2.39901, -1.6049, -0.164074};
std::vector<double> discard = {1.41853, -0.659879, 1.01533, -1.89876, -1.54768, -0.164098};

std::vector<double> mirMiddle = {2.10158, -1.53574, 2.63307, -2.63426, -1.52555, -0.164074};
std::vector<double> mir = {3.41145, -1.12724, 2.30907, -2.74562, -1.5171, -0.164086};

std::mutex configLock;
bool newTarget;
bool targetReached;

std::string currentTarget;

bool gotoConfig(lego_bricks_packing::goto_config::Request &req, lego_bricks_packing::goto_config::Response & res);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle n;

    //Service:
    ros::ServiceServer goToConfigService = n.advertiseService("go_to_config", gotoConfig);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    newTarget = false;
    targetReached = true;
    bool success;
    bool validTarget = true;

    while(ros::ok())
    {
        configLock.lock();
        if(newTarget)
        {
            validTarget = true;
            if(currentTarget.compare("pickupSmall") == 0)
                move_group.setJointValueTarget(pickupSmall);
            else if(currentTarget.compare("pickupMedium") == 0)
                move_group.setJointValueTarget(pickupMedium);
            else if(currentTarget.compare("pickupLarge") == 0)
                move_group.setJointValueTarget(pickupLarge);
            else if(currentTarget.compare("box") == 0)
                move_group.setJointValueTarget(box);
            else if(currentTarget.compare("discard") == 0)
                move_group.setJointValueTarget(discard);
            else
            {
                validTarget = false;
                ROS_ERROR("Invalid configuration name.");
            }

            if(validTarget)
            {
                ROS_INFO_STREAM("Planning to " << currentTarget);
                success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

                if (success)
                {
                    ROS_INFO_STREAM("Moving to " << currentTarget);
                    move_group.move();
                }
                else
                    ROS_ERROR("Planning failed");

                targetReached = true;
                newTarget = false;
            }
        }
        configLock.unlock();
        ros::Duration(0.1).sleep();

    }
    
    ros::spin();

    return 0;
}

bool gotoConfig(lego_bricks_packing::goto_config::Request &req, lego_bricks_packing::goto_config::Response &res)
{
    ROS_INFO("Callback");
    configLock.lock();
    if(targetReached)
    {
        newTarget = true;
        currentTarget = req.config_name;
        res.success = true;
        configLock.unlock();
        return true;
    }
    else
    {
        res.success = false;
        configLock.unlock();
        return false;
    }
    
    
}