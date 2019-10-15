#include <iostream>
#include "ros/ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

std::vector<double> pos1 = {-101.48*0.0174532925, -117.85*0.0174532925, -95.05*0.0174532925, -57,58*0.0174532925, 80.14*0.0174532925, -59.21*0.0174532925};
std::vector<double> pos2 = {-78.40*0.0174532925, -96.55*0.0174532925, -133.68*0.0174532925, -33.01*0.0174532925, 85.91*0.0174532925, -59.21*0.0174532925};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_demo");
    ros::NodeHandle n;

    std::string pg = "manipulator";

    if(argc == 2)
    {
        pg = argv[1];       
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = pg;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[5] = -1.0;  // radians
    move_group.setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_STREAM("Did planning succeed: " << success);

    if(success)
        move_group.move();


    while(ros::ok())
    {
        joint_group_positions[5] *= -1.0;  // radians
        move_group.setJointValueTarget(joint_group_positions);        

        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_STREAM("Did planning succeed: " << success);

        if(success)
            move_group.move();

        /*move_group.setJointValueTarget(pos1);

        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_STREAM("Did planning succeed: " << success);

        if(success)
            move_group.move();

        ros::Duration(1).sleep();

        move_group.setJointValueTarget(pos2);

        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_STREAM("Did planning succeed: " << success);

        if(success)
            move_group.move();

        ros::Duration(1).sleep();*/
    }

    return 0;
}