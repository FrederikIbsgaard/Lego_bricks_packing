#include <iostream>
#include "ros/ros.h"
 #include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

geometry_msgs::PoseStamped createPose(double x, double y, double z, double R, double P, double Y);
void rotate180yaw(geometry_msgs::PoseStamped& pose);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pickup_demo_no_gripper");
    ros::NodeHandle n;

    //Poses:
    geometry_msgs::PoseStamped preGrasp1 = createPose(-0.3097, -0.4345, 0.1795, 0.724, 3.305, -0.069);
    geometry_msgs::PoseStamped grasp1 = createPose(-0.3097, -0.4345, 0.06, 0.724, 3.305, -0.069);

    geometry_msgs::PoseStamped preGrasp2 = createPose(0.1558, -0.4065, 0.1795, 0.717, -3.105, -0.064);
    geometry_msgs::PoseStamped grasp2 = createPose(0.1558, -0.4065, 0.06, 0.717, -3.105, -0.064);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

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
    
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    geometry_msgs::PoseStamped current_pose;
    tf2::Matrix3x3 m;
    double roll, pitch, yaw;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = false;

    ROS_INFO_STREAM("move_group.getPlanningFrame(): " << move_group.getPlanningFrame());
    bool setEeSuccess = move_group.setEndEffectorLink("ee_link");
    ROS_INFO_STREAM("move_group.getEndEffectorLink(): " << move_group.getEndEffectorLink());

    ROS_INFO_STREAM("Links:");

    std::vector<std::string> links = move_group.getLinkNames();

    for(int i = 0; i < links.size(); i++)
    {
        ROS_INFO_STREAM(links[i]);
    }

    moveit::core::Transforms::getAllTransforms();

    while(ros::ok())
    {
        //preGrasp1:
        move_group.setPoseTarget(preGrasp1);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success)
        {
            ROS_INFO("Moving to pre-grasp 1");
            ROS_INFO_STREAM(preGrasp1);
            move_group.move();
        }
        else
            break;

        ros::Duration(1).sleep();

        ROS_INFO("Press ENTER to continue");
        /*DEBUG*/ std::cin.get();

        //grasp1:
        move_group.setPoseTarget(grasp1);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success)
        {
            ROS_INFO("Moving to grasp 1");
            ROS_INFO_STREAM(grasp1);
            move_group.move();
        }
        else
            break;

        ros::Duration(1).sleep();
        ROS_INFO("Press ENTER to continue");
        /*DEBUG*/ std::cin.get();

        //preGrasp2:
        move_group.setPoseTarget(preGrasp2);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success)
        {
            ROS_INFO("Moving to pre-grasp 2");
            ROS_INFO_STREAM(preGrasp2);
            move_group.move();
        }
        else
            break;

        ros::Duration(1).sleep();
        ROS_INFO("Press ENTER to continue");
        /*DEBUG*/ std::cin.get();

        //grasp2:
        move_group.setPoseTarget(grasp2);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success)
        {
            ROS_INFO("Moving to grasp 2");
            ROS_INFO_STREAM(grasp2);
            move_group.move();
        }
        else
            break;

        ros::Duration(1).sleep();
        ROS_INFO("Press ENTER to continue");
        /*DEBUG*/ std::cin.get();


    }

    ROS_ERROR("Planning failed...");

    return 0;
}

geometry_msgs::PoseStamped createPose(double x, double y, double z, double R, double P, double Y)
{
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(R, P, Y);

    pose.pose.orientation = tf2::toMsg(q);

    return pose;
}

void rotate180yaw(geometry_msgs::PoseStamped& pose)
{
    tf2::Quaternion qRot;
    qRot.setRPY(0.0, 0.0, 3.14159);

    tf2::Quaternion qOrig(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

    qOrig = qRot*qOrig;
    qOrig.normalize();

    pose.pose.orientation = tf2::toMsg(qOrig);
}