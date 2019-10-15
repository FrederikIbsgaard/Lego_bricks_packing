#include <iostream>
#include "ros/ros.h"
 #include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "print_pose_test");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    geometry_msgs::PoseStamped current_pose;
    tf2::Quaternion q;
    tf2::Matrix3x3 m;
    double roll, pitch, yaw;

    while(ros::ok())
    {
        current_pose = move_group.getCurrentPose("wrist_3_link");
        q = tf2::Quaternion(current_pose.pose.orientation.x, current_pose.pose.orientation.y,current_pose.pose.orientation.z, current_pose.pose.orientation.w);
        m = tf2::Matrix3x3(q);
        m.getRPY(roll, pitch, yaw);
        ROS_INFO_STREAM("x: " << current_pose.pose.position.x << " y: " << current_pose.pose.position.y << " z: " << current_pose.pose. position.z << " R: " << roll << " P: " << pitch << " Y:" << yaw);

        ros::Duration(1).sleep();
    }

    return 0;
}