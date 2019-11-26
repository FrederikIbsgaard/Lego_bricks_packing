#include <iostream>
#include "ros/ros.h"
#include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "robot_control/goto_config.h"
#include "configsAndPlans.h"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <ur_msgs/SetSpeedSliderFraction.h>

#include <ur_msgs/SetIO.h>

#include <mutex>

#define MAX_SPEED 0.3

std::mutex configLock;
std::mutex robotMovingLock;
bool newTarget;
bool targetReached;

std::string currentTarget;

moveit::planning_interface::MoveGroupInterface* mgPtr;
ros::ServiceClient* gripperPtr;
ros::ServiceClient* pausePtr;

std::string pausedTarget;
bool isPaused = false;
std::mutex pauseLock;

bool gotoConfig(robot_control::goto_config::Request &req, robot_control::goto_config::Response & res);

bool initPaths(moveit::planning_interface::MoveGroupInterface &move_group);

void pauseRobot(const std_msgs::Empty::ConstPtr& msg);

void playRobot(const std_msgs::Empty::ConstPtr& msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle n;

    //Service:
    ros::ServiceServer goToConfigService = n.advertiseService("go_to_config", gotoConfig);

    ros::AsyncSpinner spinner(3);
    spinner.start();

    //MoveIt! interface:
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    mgPtr = &move_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //Set workspace bounds:
    move_group.setWorkspace(-1, -1, 0.05, 1, 1, 0.6);

    //Msg to open and close gripper:
    ros::ServiceClient urIoClient = n.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");
    gripperPtr = & urIoClient;

    //Sub. to pause-topic:
    ros::Subscriber pauseSub = n.subscribe("/gui_pause", 1, pauseRobot);
    ros::ServiceClient pauseClient = n.serviceClient<ur_msgs::SetSpeedSliderFraction>("/ur_hardware_interface/set_speed_slider");
    pausePtr = &pauseClient;

    //Sub. to play-topic:
    ros::Subscriber playSub = n.subscribe("/gui_play", 1, playRobot);
    
    while(ros::master::check() && ros::ok());

    return 0;
}

bool gotoConfig(robot_control::goto_config::Request &req, robot_control::goto_config::Response &res)
{
    //Wait for robot to unpause:
    bool pauseCopy = true;

    while(pauseCopy)
    {
        pauseLock.lock();
        pauseCopy = isPaused;
        pauseLock.unlock();
        ros::Duration(0.1).sleep();
    }

    ROS_INFO_STREAM("pauseCopy: " << pauseCopy);

    //IO message for opening/closing gripper:
    ur_msgs::SetIO gripper;
    gripper.request.fun = 1;
    gripper.request.pin = 4;
    gripper.request.state = 1.0;

    //For planning:
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool validTarget = true;
    
    //Save target, in case of pausing:
    configLock.lock();
    currentTarget = req.config_name;
    pausedTarget = currentTarget;
    configLock.unlock();

    if(currentTarget.compare("graspSmall") == 0)
        mgPtr->setJointValueTarget(graspSmall);
    else if(currentTarget.compare("graspMedium") == 0)
        mgPtr->setJointValueTarget(graspMedium);
    else if(currentTarget.compare("graspLarge") == 0)
        mgPtr->setJointValueTarget(graspLarge);
    else if(currentTarget.compare("preGraspSmall") == 0)
        mgPtr->setJointValueTarget(preGraspSmall);
    else if(currentTarget.compare("preGraspMedium") == 0)
        mgPtr->setJointValueTarget(preGraspMedium);
    else if(currentTarget.compare("preGraspLarge") == 0)
        mgPtr->setJointValueTarget(preGraspLarge);
    else if(currentTarget.compare("aboveBoxA") == 0)
        mgPtr->setJointValueTarget(aboveBoxA);
    else if(currentTarget.compare("aboveBoxB") == 0)
        mgPtr->setJointValueTarget(aboveBoxB);
    else if(currentTarget.compare("aboveBoxC") == 0)
        mgPtr->setJointValueTarget(aboveBoxC);
    else if(currentTarget.compare("aboveBoxD") == 0)
        mgPtr->setJointValueTarget(aboveBoxD);
    else if(currentTarget.compare("aboveDiscard") == 0)
        mgPtr->setJointValueTarget(aboveDiscard);
    else
    {
        ROS_ERROR("Invalid configuration name. Not moving.");
        res.success = false;
        return false;
    }

    ROS_INFO_STREAM("Planning to " << currentTarget);
    if(mgPtr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO_STREAM("Moving to " << currentTarget);
        mgPtr->move();
        ROS_INFO("Finished moving.");

        //Check whether to open or close the gripper, depending on the conf.:
        ROS_INFO("***********************CHECK IF GRIPPER SHOULD CLOSE***********************");
        if(currentTarget.compare("aboveDiscard") == 0 || currentTarget.compare("aboveBoxA") == 0 || currentTarget.compare("aboveBoxB") == 0)
        {
            //Open the gripper:
            ROS_INFO("Opening the gripper");
            gripper.request.state = 0.0;
            if(!gripperPtr->call(gripper))
            {
                ROS_ERROR("Failed to contact gripper");
            }
        }
        else if(currentTarget.compare("aboveBoxC") == 0 || currentTarget.compare("aboveBoxD") == 0)
        {
            //Open the gripper:
            ROS_INFO("Opening the gripper");
            gripper.request.state = 0.0;
            if(!gripperPtr->call(gripper))
            {
                ROS_ERROR("Failed to contact gripper");
            }
        }
        else if(currentTarget.compare("graspSmall") == 0 || currentTarget.compare("graspMedium") == 0 || currentTarget.compare("graspLarge") == 0)
        {
            //Close the gripper:
            ROS_INFO("Closing the gripper");
            gripper.request.state = 1.0;
            if(!gripperPtr->call(gripper))
            {
                ROS_ERROR("Failed to contact gripper");
            }
        }
        
        res.success = true;
        return true;      
    }
    else
    {
        ROS_ERROR("Planning failed");
        res.success = false;
        return false;
    }
}

void pauseRobot(const std_msgs::Empty::ConstPtr& msg)
{
    ROS_INFO("PAUSE");
    pauseLock.lock();
    ROS_INFO("LOCK");
    isPaused = true;
    pauseLock.unlock();
    
    configLock.lock();
    ROS_INFO_STREAM("Paused target: " << pausedTarget);
    configLock.unlock();

    //Slow down robot:
    ur_msgs::SetSpeedSliderFraction speed;
    speed.request.speed_slider_fraction = 0.01;

    if (!pausePtr->call(speed))
    {
        ROS_ERROR_STREAM("Failed to stop robot! Speed: " << 0.01);
    }

    if (!speed.response.success)
    {
        ROS_ERROR_STREAM("Service call succeeded, but failed to stop robot! Speed: " << 0.01);
    }

    //Stop-hack: Plan to current config and execute:
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    std::vector<double> q;
    q = mgPtr->getCurrentJointValues();
    mgPtr->setJointValueTarget(q);
    if(mgPtr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        mgPtr->move();
        ROS_INFO("Robot stopped");
    }
    else
    {
        ROS_ERROR("Failed to stop robot!");
    }
}

void playRobot(const std_msgs::Empty::ConstPtr& msg)
{
    ROS_INFO("PLAY");
    //Speed up robot:
    ur_msgs::SetSpeedSliderFraction speed;

    speed.request.speed_slider_fraction = MAX_SPEED;

    if (!pausePtr->call(speed))
    {
        ROS_ERROR_STREAM("Failed to set speed: " << MAX_SPEED);
    }

    if (!speed.response.success)
    {
        ROS_ERROR_STREAM("Service call succeeded, but failed to set speed: " << MAX_SPEED);
    }

    ROS_INFO("Getting paused copy");
    configLock.lock();
    std::string pausedTargetCopy = pausedTarget;
    configLock.unlock();
    ROS_INFO_STREAM("Paused target: " << pausedTargetCopy);

    pauseLock.lock();
    //IO message for opening/closing gripper:
    ur_msgs::SetIO gripper;
    gripper.request.fun = 1;
    gripper.request.pin = 4;
    gripper.request.state = 1.0;


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    if(pausedTargetCopy.compare("graspSmall") == 0)
        mgPtr->setJointValueTarget(graspSmall);
    else if(pausedTargetCopy.compare("graspMedium") == 0)
        mgPtr->setJointValueTarget(graspMedium);
    else if(pausedTargetCopy.compare("graspLarge") == 0)
        mgPtr->setJointValueTarget(graspLarge);
    else if(pausedTargetCopy.compare("preGraspSmall") == 0)
        mgPtr->setJointValueTarget(preGraspSmall);
    else if(pausedTargetCopy.compare("preGraspMedium") == 0)
        mgPtr->setJointValueTarget(preGraspMedium);
    else if(pausedTargetCopy.compare("preGraspLarge") == 0)
        mgPtr->setJointValueTarget(preGraspLarge);
    else if(pausedTargetCopy.compare("aboveBoxA") == 0)
        mgPtr->setJointValueTarget(aboveBoxA);
    else if(pausedTargetCopy.compare("aboveBoxB") == 0)
        mgPtr->setJointValueTarget(aboveBoxB);
    else if(pausedTargetCopy.compare("aboveBoxC") == 0)
        mgPtr->setJointValueTarget(aboveBoxC);
    else if(pausedTargetCopy.compare("aboveBoxD") == 0)
        mgPtr->setJointValueTarget(aboveBoxD);
    else if(pausedTargetCopy.compare("aboveDiscard") == 0)
        mgPtr->setJointValueTarget(aboveDiscard);
    else
    {
        ROS_ERROR("Invalid configuration name. Not moving.");
        return;
    }

    ROS_INFO_STREAM("Re-planning to " << pausedTarget);
    if(mgPtr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO_STREAM("Moving to " << pausedTarget);
        mgPtr->move();
        ROS_INFO("Finished moving.");

        //Check whether to open or close the gripper, depending on the conf.:
        ROS_INFO("***********************CHECK IF GRIPPER SHOULD CLOSE***********************");
        if(pausedTarget.compare("aboveDiscard") == 0 || pausedTarget.compare("aboveBoxA") == 0 || pausedTarget.compare("aboveBoxB") == 0)
        {
            //Open the gripper:
            ROS_INFO("Opening the gripper");
            gripper.request.state = 0.0;
            if(!gripperPtr->call(gripper))
            {
                ROS_ERROR("Failed to contact gripper");
            }
        }
        else if(pausedTarget.compare("aboveBoxC") == 0 || pausedTarget.compare("aboveBoxD") == 0)
        {
            //Open the gripper:
            ROS_INFO("Opening the gripper");
            gripper.request.state = 0.0;
            if(!gripperPtr->call(gripper))
            {
                ROS_ERROR("Failed to contact gripper");
            }
        }
        else if(pausedTarget.compare("graspSmall") == 0 || pausedTarget.compare("graspMedium") == 0 || pausedTarget.compare("graspLarge") == 0)
        {
            //Close the gripper:
            ROS_INFO("Closing the gripper");
            gripper.request.state = 1.0;
            if(!gripperPtr->call(gripper))
            {
                ROS_ERROR("Failed to contact gripper");
            }
        }

        configLock.lock();
        pausedTarget = "";
        configLock.unlock();      
    }
    else
    {
        ROS_ERROR("Planning failed");
        return;
    }

    isPaused = false;
    pauseLock.unlock();
}