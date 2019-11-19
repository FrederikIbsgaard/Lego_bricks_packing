#include <iostream>
#include <pluginlib/class_loader.h>
#include "ros/ros.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/MotionPlanResponse.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include<geometry_msgs/Pose.h>

#include <boost/scoped_ptr.hpp>

#include "configsAndPlans.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_test");
    ros::NodeHandle n;

    //Service:
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    bool success;
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    moveit::planning_interface::MoveGroupInterface::Plan plan2;

    // geometry_msgs::Pose currentPose = move_group.getCurrentPose().pose;

    // ROS_INFO_STREAM(currentPose);

    //Poses:
    geometry_msgs::Pose aboveBoxA;
    aboveBoxA.position.x = 0.0818172;
    aboveBoxA.position.y = 0.383145;
    aboveBoxA.position.z = 0.22495;
    aboveBoxA.orientation.x =  0.130755;
    aboveBoxA.orientation.y = 0.718847;
    aboveBoxA.orientation.z = -0.118472;
    aboveBoxA.orientation.w = 0.672403;

    geometry_msgs::Pose preGraspLarge;
    preGraspLarge.position.x = 0.103606;
    preGraspLarge.position.y = 0.457402;
    preGraspLarge.position.z = 0.252574;
    preGraspLarge.orientation.x = 0.382121;
    preGraspLarge.orientation.y = 0.601011;
    preGraspLarge.orientation.z = 0.123616;
    preGraspLarge.orientation.w = 0.691006;

    geometry_msgs::Pose graspLarge;
    graspLarge.position.x = 0.139343;
    graspLarge.position.y = 0.521596;
    graspLarge.position.z = 0.201991;
    graspLarge.orientation.x = 0.365753;
    graspLarge.orientation.y = 0.615832;
    graspLarge.orientation.z = 0.104855;
    graspLarge.orientation.w = 0.689914;

    std::vector<geometry_msgs::Pose> waypoints = {aboveBoxA, preGraspLarge, graspLarge};

    moveit_msgs::RobotTrajectory t;
    double res = move_group.computeCartesianPath(waypoints, 0.1, 0.01, t);
    ROS_INFO_STREAM("res: " << res);

    if(res != -1.0)
    {
        plan1.trajectory_ = t;
        move_group.execute(plan1);
    }

    //success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //if(success)
      //  move_group.move();


    // planning_interface::MotionPlanRequest req;
    // planning_interface::MotionPlanResponse res;

    // ros::AsyncSpinner spinner(1);
    // spinner.start();

    // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    // robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    // /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
    // robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    // const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("manipulator");

    // planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    // planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

    //  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    // planning_interface::PlannerManagerPtr planner_instance;
    // std::string planner_plugin_name;

    // // We will get the name of planning plugin we want to load
    // // from the ROS parameter server, and then load the planner
    // // making sure to catch all exceptions.
    // if (!n.getParam("/move_group/planning_plugin", planner_plugin_name))
    //     ROS_FATAL_STREAM("Could not find planner plugin name");
    // try
    // {
    //     planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
    //         "moveit_core", "planning_interface::PlannerManager"));
    // }
    // catch (pluginlib::PluginlibException& ex)
    // {
    //     ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    // }
    // try
    // {
    //     planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    //     if (!planner_instance->initialize(robot_model, n.getNamespace()))
    //     ROS_FATAL_STREAM("Could not initialize planner instance");
    //     ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    // }
    // catch (pluginlib::PluginlibException& ex)
    // {
    //     const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    //     std::stringstream ss;
    //     for (std::size_t i = 0; i < classes.size(); ++i)
    //     ss << classes[i] << " ";
    //     ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
    //                                                         << "Available plugins: " << ss.str());
    // }

    // robot_state::RobotState goal_state(robot_model);
    // std::vector<double> preGraspLarge = {1.03776, -1.38167, 2.33591, -3.26135, -1.45834, 1.48253};
    // goal_state.setJointGroupPositions(joint_model_group, preGraspLarge);
    // moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    // req.goal_constraints.clear();
    // req.goal_constraints.push_back(joint_goal);


    // std::vector<double> graspLarge = {1.04001, -1.1492, 2.16316, -3.26365, -1.45432, 1.4825};
    // robot_state::RobotState waypoint(robot_model);
    // waypoint.setJointGroupPositions(joint_model_group, graspLarge);
    // moveit_msgs::Constraints joint_waypoint = kinematic_constraints::constructGoalConstraints(waypoint, joint_model_group);
    // req.goal_constraints.push_back(joint_waypoint);


    // //Planning:
    // req.group_name = "manipulator";
    // planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    // context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    // /* Call the Planner */
    // context->solve(res);
    // /* Check that the planning was successful */
    // if (res.error_code_.val != res.error_code_.SUCCESS)
    // {
    //     ROS_ERROR("Could not compute plan successfully");
    //     return 0;
    // }
    // else
    // {
    //     ROS_INFO("Planning success!");
    // }

    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // my_plan.planning_time_ = res.planning_time_;
    // my_plan.trajectory_ = res.trajectory_;

    // static const std::string PLANNING_GROUP = "manipulator";
    // moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // ROS_INFO("Moving!");
    // move_group.execute(my_plan);    

    return 0;
}