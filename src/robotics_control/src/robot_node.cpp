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

#include <ur_msgs/SetIO.h>

#include <mutex>

std::mutex configLock;
std::mutex robotMovingLock;
bool newTarget;
bool targetReached;

std::string currentTarget;

bool gotoConfig(robot_control::goto_config::Request &req, robot_control::goto_config::Response & res);

bool initPaths(moveit::planning_interface::MoveGroupInterface &move_group);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle n;

    //Service:
    ros::ServiceServer goToConfigService = n.advertiseService("go_to_config", gotoConfig);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    bool success;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    //Set workspace bounds:
    move_group.setWorkspace(-1, -1, 0.05, 1, 1, 0.6);

    //Run save-plans-routine:
    /*if(!initPaths(move_group))
    {
        ROS_ERROR("Initialization failed, please read error-messages and retry.");
        return -1;
    }*/
   

    //Msg to open and close gripper:
    ros::ServiceClient urIoClient = n.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");
    ur_msgs::SetIO gripper;
    gripper.request.fun = 1;
    gripper.request.pin = 4;
    gripper.request.state = 1.0;

    newTarget = false;
    targetReached = true;
    bool validTarget = true;

    while(ros::ok())
    {
        configLock.lock();
        if(newTarget)
        {
            validTarget = true;
            if(currentTarget.compare("graspSmall") == 0)
                move_group.setJointValueTarget(graspSmall);
            else if(currentTarget.compare("graspMedium") == 0)
                move_group.setJointValueTarget(graspMedium);
            else if(currentTarget.compare("graspLarge") == 0)
                move_group.setJointValueTarget(graspLarge);
            else if(currentTarget.compare("preGraspSmall") == 0)
                move_group.setJointValueTarget(preGraspSmall);
            else if(currentTarget.compare("preGraspMedium") == 0)
                move_group.setJointValueTarget(preGraspMedium);
            else if(currentTarget.compare("preGraspLarge") == 0)
                move_group.setJointValueTarget(preGraspLarge);
            else if(currentTarget.compare("aboveBoxA") == 0)
                move_group.setJointValueTarget(aboveBoxA);
            else if(currentTarget.compare("aboveDiscard") == 0)
                move_group.setJointValueTarget(aboveDiscard);
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
                    ROS_INFO("Finished moving.");

                    //Check whether to open or close the gripper, depending on the conf.:
                    if(currentTarget.compare("aboveDiscard") == 0 || currentTarget.compare("aboveBoxA") == 0)
                    {
                        //Open the gripper:
                        ROS_INFO("Opening the gripper");
                        gripper.request.state = 0.0;
                        if(!urIoClient.call(gripper))
                        {
                            ROS_ERROR("Failed to contact gripper");
                            //return -1;
                        }
                    }
                    // else if(currentTarget.compare("preGraspSmall") == 0 || currentTarget.compare("preGraspMedium") == 0 || currentTarget.compare("preGraspLarge") == 0)
                    // {
                    //     //Open the gripper:
                    //     ROS_INFO("Opening the gripper");
                    //     gripper.request.state = 0.0;
                    //     if(!urIoClient.call(gripper))
                    //     {
                    //         ROS_ERROR("Failed to contact gripper");
                    //         return -1;
                    //     }
                    // }
                    else if(currentTarget.compare("graspSmall") == 0 || currentTarget.compare("graspMedium") == 0 || currentTarget.compare("graspLarge") == 0)
                    {
                        //Close the gripper:
                        ROS_INFO("Closing the gripper");
                        gripper.request.state = 1.0;
                        if(!urIoClient.call(gripper))
                        {
                            ROS_ERROR("Failed to contact gripper");
                            //return -1;
                        }
                    }
                    
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
    return 0;
}

bool gotoConfig(robot_control::goto_config::Request &req, robot_control::goto_config::Response &res)
{
    ROS_INFO("Receiving new goal.");
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

// bool initPaths(moveit::planning_interface::MoveGroupInterface &move_group)
// {
//     moveit::planning_interface::MoveGroupInterface::Plan tempPlan;
//     bool success;

//     ROS_INFO("Running intialization routine. Planning to idle-config.");
//     move_group.setJointValueTarget(idleConfig);
//     success = (move_group.plan(forwardPlans::firstPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Could not plan to idle-configuration. Please manually move robot closer, and try again.");
//         return false;
//     }

//     /*
//      * Pre-grasp to grasp
//      */
//     //---------------------------------------------------
//     ROS_INFO("Planning to pre-grasp small.");
//     move_group.setJointValueTarget(preGraspSmall);
//     success = (move_group.plan(forwardPlans::idleToPreSmall) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to idle.");
//     move_group.setJointValueTarget(idleConfig);
//     success = (move_group.plan(reversePlans::idleToPreSmall) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //---------------------------------------------------
//     ROS_INFO("Planning to pre-grasp medium.");
//     move_group.setJointValueTarget(preGraspMedium);
//     success = (move_group.plan(forwardPlans::idleToPreMedium) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to idle.");
//     move_group.setJointValueTarget(idleConfig);
//     success = (move_group.plan(reversePlans::idleToPreMedium) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }
//     return true;

//     //---------------------------------------------------
//     ROS_INFO("Planning to pre-grasp large.");
//     move_group.setJointValueTarget(preGraspLarge);
//     success = (move_group.plan(forwardPlans::idleToPreLarge) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to idle.");
//     move_group.setJointValueTarget(idleConfig);
//     success = (move_group.plan(reversePlans::idleToPreLarge) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //---------------------------------------------------
//     ROS_INFO("Moving to pre-grasp small.");
//     if(move_group.execute(forwardPlans::idleToPreSmall) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
//     {
//         ROS_ERROR("Executing failed.");
//         return false;
//     }

//     ROS_INFO("Planning to grasp small.");
//     move_group.setJointValueTarget(graspSmall);
//     success = (move_group.plan(forwardPlans::graspPreSmall) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to preGraspSmall.");
//     move_group.setJointValueTarget(preGraspSmall);
//     success = (move_group.plan(reversePlans::graspPreSmall) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //---------------------------------------------------
//     ROS_INFO("Moving to idle.");
//     if(move_group.execute(reversePlans::idleToPreSmall) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
//     {
//         ROS_ERROR("Executing failed.");
//         return false;
//     }

//     ROS_INFO("Moving to pre-grasp medium.");
//     if(move_group.execute(forwardPlans::idleToPreMedium) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
//     {
//         ROS_ERROR("Executing failed.");
//         return false;
//     }

//     ROS_INFO("Planning to grasp medium.");
//     move_group.setJointValueTarget(graspMedium);
//     success = (move_group.plan(forwardPlans::graspPreMedium) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to preGraspMedium.");
//     move_group.setJointValueTarget(preGraspMedium);
//     success = (move_group.plan(reversePlans::graspPreMedium) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //---------------------------------------------------
//     ROS_INFO("Moving to idle.");
//     if(move_group.execute(reversePlans::idleToPreMedium) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
//     {
//         ROS_ERROR("Executing failed.");
//         return false;
//     }

//     ROS_INFO("Moving to pre-grasp large.");
//     if(move_group.execute(forwardPlans::idleToPreLarge) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
//     {
//         ROS_ERROR("Executing failed.");
//         return false;
//     }

//     ROS_INFO("Planning to grasp large.");
//     move_group.setJointValueTarget(graspLarge);
//     success = (move_group.plan(forwardPlans::graspPreLarge) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to preGraspLarge.");
//     move_group.setJointValueTarget(preGraspLarge);
//     success = (move_group.plan(reversePlans::graspPreLarge) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     /*
//      * Pre-grasp to boxes
//      */
//     //---------------------------------------------------
//     ROS_INFO("Planning to box A.");
//     move_group.setJointValueTarget(aboveBoxA);
//     success = (move_group.plan(forwardPlans::largeToBoxA) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to preGraspLarge.");
//     move_group.setJointValueTarget(preGraspLarge);
//     success = (move_group.plan(reversePlans::largeToBoxA) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //---------------------------------------------------
//     ROS_INFO("Planning to box B.");
//     move_group.setJointValueTarget(aboveBoxB);
//     success = (move_group.plan(forwardPlans::largeToBoxB) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to preGraspLarge.");
//     move_group.setJointValueTarget(preGraspLarge);
//     success = (move_group.plan(reversePlans::largeToBoxB) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //---------------------------------------------------
//     ROS_INFO("Planning to box C.");
//     move_group.setJointValueTarget(aboveBoxC);
//     success = (move_group.plan(forwardPlans::largeToBoxC) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to preGraspLarge.");
//     move_group.setJointValueTarget(preGraspLarge);
//     success = (move_group.plan(reversePlans::largeToBoxC) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //---------------------------------------------------
//     ROS_INFO("Planning to box D.");
//     move_group.setJointValueTarget(aboveBoxD);
//     success = (move_group.plan(forwardPlans::largeToBoxD) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to preGraspLarge.");
//     move_group.setJointValueTarget(preGraspLarge);
//     success = (move_group.plan(reversePlans::largeToBoxD) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //---------------------------------------------------
//     ROS_INFO("Moving to preGraspMedium.");
//     move_group.setJointValueTarget(preGraspMedium);
//     success = (move_group.plan(tempPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning to box A.");
//     move_group.setJointValueTarget(aboveBoxA);
//     success = (move_group.plan(forwardPlans::mediumToBoxA) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to preGraspMedium.");
//     move_group.setJointValueTarget(preGraspMedium);
//     success = (move_group.plan(reversePlans::mediumToBoxA) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //---------------------------------------------------
//     ROS_INFO("Planning to box B.");
//     move_group.setJointValueTarget(aboveBoxB);
//     success = (move_group.plan(forwardPlans::mediumToBoxB) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to preGraspMedium.");
//     move_group.setJointValueTarget(preGraspMedium);
//     success = (move_group.plan(reversePlans::mediumToBoxB) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //---------------------------------------------------
//     ROS_INFO("Planning to box C.");
//     move_group.setJointValueTarget(aboveBoxC);
//     success = (move_group.plan(forwardPlans::mediumToBoxC) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to preGraspMedium.");
//     move_group.setJointValueTarget(preGraspMedium);
//     success = (move_group.plan(reversePlans::mediumToBoxC) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //---------------------------------------------------
//     ROS_INFO("Planning to box D.");
//     move_group.setJointValueTarget(aboveBoxD);
//     success = (move_group.plan(forwardPlans::mediumToBoxD) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to preGraspMedium.");
//     move_group.setJointValueTarget(preGraspMedium);
//     success = (move_group.plan(reversePlans::mediumToBoxD) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //---------------------------------------------------
//     ROS_INFO("Moving to preGraspSmall.");
//     move_group.setJointValueTarget(preGraspSmall);
//     success = (move_group.plan(tempPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning to box A.");
//     move_group.setJointValueTarget(aboveBoxA);
//     success = (move_group.plan(forwardPlans::smallToBoxA) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to preGraspSmall.");
//     move_group.setJointValueTarget(preGraspMedium);
//     success = (move_group.plan(reversePlans::smallToBoxA) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //---------------------------------------------------
//     ROS_INFO("Planning to box B.");
//     move_group.setJointValueTarget(aboveBoxB);
//     success = (move_group.plan(forwardPlans::smallToBoxB) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to preGraspSmall.");
//     move_group.setJointValueTarget(preGraspMedium);
//     success = (move_group.plan(reversePlans::smallToBoxB) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //---------------------------------------------------
//     ROS_INFO("Planning to box C.");
//     move_group.setJointValueTarget(aboveBoxC);
//     success = (move_group.plan(forwardPlans::smallToBoxC) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to preGraspSmall.");
//     move_group.setJointValueTarget(preGraspMedium);
//     success = (move_group.plan(reversePlans::smallToBoxC) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //---------------------------------------------------
//     ROS_INFO("Planning to box D.");
//     move_group.setJointValueTarget(aboveBoxD);
//     success = (move_group.plan(forwardPlans::smallToBoxD) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to preGraspSmall.");
//     move_group.setJointValueTarget(preGraspMedium);
//     success = (move_group.plan(reversePlans::smallToBoxD) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //---------------------------------------------------
//     ROS_INFO("Planning to discard box.");
//     move_group.setJointValueTarget(aboveDiscard);
//     success = (move_group.plan(forwardPlans::smallToDiscard) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to preGraspSmall.");
//     move_group.setJointValueTarget(preGraspSmall);
//     success = (move_group.plan(reversePlans::smallToDiscard) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //---------------------------------------------------
//     ROS_INFO("Moving to preGraspMedium.");
//     move_group.setJointValueTarget(preGraspMedium);
//     success = (move_group.plan(tempPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning to discard box.");
//     move_group.setJointValueTarget(aboveDiscard);
//     success = (move_group.plan(forwardPlans::mediumToDiscard) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to preGraspMedium.");
//     move_group.setJointValueTarget(preGraspMedium);
//     success = (move_group.plan(reversePlans::smallToDiscard) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //---------------------------------------------------
//     ROS_INFO("Moving to preGraspLarge.");
//     move_group.setJointValueTarget(preGraspLarge);
//     success = (move_group.plan(tempPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning to discard box.");
//     move_group.setJointValueTarget(aboveDiscard);
//     success = (move_group.plan(forwardPlans::largeToDiscard) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     ROS_INFO("Planning back to preGraspLarge.");
//     move_group.setJointValueTarget(preGraspLarge);
//     success = (move_group.plan(reversePlans::largeToDiscard) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(success)
//         move_group.move();
//     else
//     {
//         ROS_ERROR("Planning failed.");
//         return false;
//     }

//     //Finshed, yay!
//     return true;
// }