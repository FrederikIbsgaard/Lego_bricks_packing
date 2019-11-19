#include "ros/ros.h"
#include <std_msgs/String.h>
#include "robot_control/goto_config.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

std::vector<double> one = {-1.97082, -1.55122, -1.40085, -0.984906, 1.53972, 1.94438};
std::vector<double> two = {-0.860782, -1.39296, -1.35189, -0.984966, 1.53974, 1.94447};
std::vector<double> three = {-1.45554, -1.32642, -1.83384, -1.39, 1.53977, 1.94446};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "e_stop_test");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    bool success;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    while(true)
    {
        move_group.setJointValueTarget(one);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if(success)
            move_group.move();
        else
        {
           ROS_ERROR("ERROR");
           return -1;
        }

        move_group.setJointValueTarget(two);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if(success)
            move_group.move();
        else
        {
           ROS_ERROR("ERROR");
           return -1;
        }

        move_group.setJointValueTarget(three);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if(success)
            move_group.move();
        else
        {
           ROS_ERROR("ERROR");
           return -1;
        }
        
    }

    return 0;
}