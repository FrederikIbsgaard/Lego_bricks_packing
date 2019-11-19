#include <iostream>
#include "ros/ros.h"
 #include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pickup_demo_no_gripper");
    ros::NodeHandle n;

    //Poses:
    std::vector<double> pickup = {1.17125, -0.811894, 1.30296, -2.49292, -1.60823, 0.0158257};
    std::vector<double> box = {1.10144, -1.02262, 1.74628, -2.27726, -1.55929, 0.0158858};
    std::vector<double> pickup2 = {0.858479, -0.837565, 1.31525, -2.37817, -1.47703, -0.234145};
    std::vector<double> discard = {1.07089, -0.911601, 1.26705, -1.93759, -1.47984, -0.233678};

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    // move_group.setPlanningTime(3.0);

    //Add table plane:
    /*ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    moveit_msgs::AttachedCollisionObject tablePlane;
    tablePlane.link_name = "wrist_3_link";
    tablePlane.object.header.frame_id = "wrist_3_link";
    tablePlane.object.id = "table_plane";
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = -5.0;
    pose.position.y = -5.0;
    pose.position.z = 0;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.01;
    primitive.dimensions[1] = 10.0;
    primitive.dimensions[2] = 10.0;

    tablePlane.object.primitives.push_back(primitive);
    tablePlane.object.primitive_poses.push_back(pose);
    tablePlane.object.operation = tablePlane.object.ADD;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(tablePlane.object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);*/
    
    //moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = false;

    while(ros::ok())
    {
        //Pickup 1:
        ROS_INFO_STREAM("Planning to pickup 1");
        move_group.setJointValueTarget(pickup);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if(success)
        {
            ROS_INFO_STREAM("Moving to pickup 1");
            move_group.move();
        }

        // Box:
        ROS_INFO_STREAM("Planning to box");
        move_group.setJointValueTarget(box);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if(success)
        {
            ROS_INFO_STREAM("Moving to box");
            move_group.move();
        }

        // Pickup 2:
        ROS_INFO_STREAM("Planning to pickup2");
        move_group.setJointValueTarget(pickup2);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if(success)
        {
            ROS_INFO_STREAM("Moving to pickup2");
            move_group.move();
        }

        // Discard:
        ROS_INFO_STREAM("Planning to discard");
        move_group.setJointValueTarget(discard);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if(success)
        {
            ROS_INFO_STREAM("Moving to discard");
            move_group.move();
        }
    }
    ROS_ERROR("Planning failed...");

    return 0;
}