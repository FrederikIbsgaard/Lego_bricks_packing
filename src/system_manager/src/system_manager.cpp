#include <iostream>
#include "ros/ros.h"

#include "state.h"
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>

#include <ur_msgs/SetIO.h>

//Services:
#include "robot_control/goto_config.h"
#include "vision/check_brick.h"
#include "mes_ordering/DeleteOrder_srv.h"
#include "mes_ordering/GetOrder_srv.h"

//Topics:


using namespace std;

#define BLUE_BRICKS   0
#define RED_BRICKS    1
#define YELLOW_BRICKS 2

#define BRICK_NOMATCH 0
#define BRICK_MATCH   1

#define FEEDER_WARNING_THRESH 5

#define FEEDER_MAX 32 //Max number of bricks in any feeder

//Order info:
int currentOrderContents[3]; //blue, red, yellow
bool boxContainsOrder[4] = {false, false, false, false};
int currentOrderId;
int currentOrderTicket;

//Estimate of feeder contents:
int feederEstimates[3] = {FEEDER_MAX, FEEDER_MAX, FEEDER_MAX};

//Runtime info:
int currentBox;
string currentBoxString;
bool running = true;

//Callbacks:
void feederRefill(const std_msgs::Empty::ConstPtr& msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "system_manager");
    ros::NodeHandle n;

    //Setup service clients:
    //Robot:
    ros::ServiceClient robotClient = n.serviceClient<robot_control::goto_config>("/go_to_config");
    robot_control::goto_config robCmd;

    //MES:
    ros::ServiceClient mesGetClient = n.serviceClient<mes_ordering::GetOrder_srv>("/MES_GetOrder");
    mes_ordering::GetOrder_srv mesCmd;

    mes_ordering::GetOrder_srv getOrder;

    ros::ServiceClient mesDelClient = n.serviceClient<mes_ordering::DeleteOrder_srv>("/MES_DeleteOrder");

    mes_ordering::DeleteOrder_srv delOrder;

    //Vision:
    ros::ServiceClient visClient = n.serviceClient<vision::check_brick>("/check_brick");
    vision::check_brick visCmd;

    //Publisher to PackML action-topic:
    ros::Publisher packmlPub = n.advertise<std_msgs::Int8>("/action_state", 5);
    std_msgs::Int8 packmlAction;

    //Publisher to feeder warning and alert:
    ros::Publisher feederWarningPub = n.advertise<std_msgs::Empty>("/feeder_warning", 1);
    ros::Publisher feederAlertPub = n.advertise<std_msgs::Empty>("/feeder_alert", 1);

    //Refill subscriber:
    ros::Subscriber feederRefillSub = n.subscribe("/feeder_refill", 1, feederRefill);

    //Gripper interface:
    //IO message for opening/closing gripper:
    ur_msgs::SetIO gripper;
    gripper.request.fun = 1;
    gripper.request.pin = 4;
    gripper.request.state = 1.0;

    ros::ServiceClient urIoClient = n.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");

    //Start packing:
    //Open the gripper:
    ROS_INFO("Opening the gripper");
    gripper.request.state = 0.0;
    if(!urIoClient.call(gripper))
    {
        ROS_ERROR("Failed to contact gripper");
    }
    
    ROS_INFO("System manager started!");
    ros::Duration(1).sleep();

    while(running)
    {
        ROS_INFO("Checking feeder status..");
        std_msgs::Empty empty;

        if(feederEstimates[BLUE_BRICKS] <= 0 || feederEstimates[RED_BRICKS] <= 0 || feederEstimates[YELLOW_BRICKS] <= 0)
        {
            feederAlertPub.publish(empty);
            ROS_INFO("Waiting for feeders to be refilled.");
            //Insert loop for waiting...
        }
        else if(feederEstimates[BLUE_BRICKS] <= FEEDER_WARNING_THRESH || feederEstimates[RED_BRICKS] <= FEEDER_WARNING_THRESH || feederEstimates[YELLOW_BRICKS] <= FEEDER_WARNING_THRESH)
        {
            feederWarningPub.publish(empty);
        }


        ROS_INFO("Asking MES system for next order...");
        getOrder.request.amount = 1;

        if(!mesGetClient.call(getOrder))
        {
            ROS_ERROR("MES servicer call failed!");
            return -1;
        }

        ROS_INFO_STREAM("Got order no. " << getOrder.response.id);
        ROS_INFO_STREAM("Ticket: " << getOrder.response.ticket);
        ROS_INFO_STREAM("Blue: " << getOrder.response.blue);
        ROS_INFO_STREAM("Red: " << getOrder.response.red);
        ROS_INFO_STREAM("Yellow: " << getOrder.response.yellow);

        currentOrderContents[BLUE_BRICKS] = getOrder.response.blue;
        currentOrderContents[RED_BRICKS] = getOrder.response.red;
        currentOrderContents[YELLOW_BRICKS] = getOrder.response.yellow;

        //Find the next available box:
        currentBox = 0;
        while (boxContainsOrder[currentBox] && currentBox < 4)
            currentBox++;

        if(currentBox == 3) //Start packing in the last box
        {
            ROS_INFO("Packing in last box, calling MiR");
            //Call MiR
        }
        else if(currentBox > 3)
        {
            ROS_INFO("Waiting for MiR...");
            //Wait for MiR
        }

        switch (currentBox)
        {
        case 0:
            currentBoxString = "aboveBoxA";
            break;
        case 1:
            currentBoxString = "aboveBoxB";
            break;
        case 2:
            currentBoxString = "aboveBoxC";
            break;
        case 3:
            currentBoxString = "aboveBoxD";
            break;
        
        default:
            ROS_ERROR("Invalid box was selected. Program error, please debug.");
            return -1;
        }

        ROS_INFO_STREAM("Packing in box " << currentBoxString);

        ROS_INFO("Packing blue bricks");
        while(currentOrderContents[BLUE_BRICKS] > 0) //Pack all the blue bricks
        {
            //Ask robot to go to preSmall, then graspSmall (auto-grasps)
            robCmd.request.config_name = "preGraspSmall";
            ROS_INFO_STREAM("Moving to " << robCmd.request.config_name);
            if(!robotClient.call(robCmd))
            {
                ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                return -1;
            }
            
            //Open the gripper:
            ROS_INFO("Opening the gripper");
            gripper.request.state = 0.0;
            if(!urIoClient.call(gripper))
            {
                ROS_ERROR("Failed to contact gripper");
            }

            robCmd.request.config_name = "graspSmall";
            ROS_INFO_STREAM("Moving to " << robCmd.request.config_name);
            if(!robotClient.call(robCmd))
            {
                ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                return -1;
            }

            //Close the gripper:
            ROS_INFO("Closing the gripper");
            gripper.request.state = 1.0;
            if(!urIoClient.call(gripper))
            {
                ROS_ERROR("Failed to contact gripper");
            }

            //Ask robot to go to preSmall
            robCmd.request.config_name = "preGraspSmall";
            ROS_INFO_STREAM("Moving to " << robCmd.request.config_name);
            if(!robotClient.call(robCmd))
            {
                ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                return -1;
            }

            //Ask vision system to validate brick:
            ros::WallTime start_ = ros::WallTime::now();
            visCmd.request.color = "blue";
            if(!visClient.call(visCmd))
            {
                ROS_ERROR("Vision service call failed!");
                return -1;
            }
            ros::WallTime end_ = ros::WallTime::now();
            double execution_time = (end_ - start_).toNSec() * 1e-6;
            ROS_INFO_STREAM("Vision time [ms]: " << execution_time);

            ROS_INFO_STREAM("Brick check result: " << (int)visCmd.response.result);
            if(visCmd.response.result == BRICK_MATCH)
            {
                //Ask robot to go to currentBoxString
                robCmd.request.config_name = currentBoxString;
                if(!robotClient.call(robCmd))
                {
                    ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                    return -1;
                }

                //Ask vision system to validate brick:
                if(!visClient.call(visCmd))
                {
                    ROS_ERROR("Vision service call failed!");
                    return -1;
                }

                if(visCmd.response.result == BRICK_MATCH)
                {
                    currentOrderContents[BLUE_BRICKS]--; //One less brick to pack :)
                    //Open the gripper:
                    ROS_INFO("Opening the gripper");
                    gripper.request.state = 0.0;
                    if(!urIoClient.call(gripper))
                    {
                        ROS_ERROR("Failed to contact gripper");
                    }
                }
                else
                {
                    ROS_ERROR("No brick to drop!");
                    //Ask robot to go to aboveDiscard
                    robCmd.request.config_name = "aboveDiscard";
                    if(!robotClient.call(robCmd))
                    {
                        ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                        return -1;
                    }

                    //Open the gripper:
                    ROS_INFO("Opening the gripper");
                    gripper.request.state = 0.0;
                    if(!urIoClient.call(gripper))
                    {
                        ROS_ERROR("Failed to contact gripper");
                    }
                }
            }
            else
            {
                //Ask robot to go to aboveDiscard
                robCmd.request.config_name = "aboveDiscard";
                if(!robotClient.call(robCmd))
                {
                    ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                    return -1;
                }

                //Open the gripper:
                ROS_INFO("Opening the gripper");
                gripper.request.state = 0.0;
                if(!urIoClient.call(gripper))
                {
                    ROS_ERROR("Failed to contact gripper");
                }
            }
        }

        ROS_INFO("Packing red bricks");
        while(currentOrderContents[RED_BRICKS] > 0) //Pack all the blue bricks
        {
            //Ask robot to go to preSmall, then graspMedium (auto-grasps)
            robCmd.request.config_name = "preGraspMedium";
            ROS_INFO_STREAM("Moving to " << robCmd.request.config_name);
            if(!robotClient.call(robCmd))
            {
                ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                return -1;
            }

            //Open the gripper:
            ROS_INFO("Opening the gripper");
            gripper.request.state = 0.0;
            if(!urIoClient.call(gripper))
            {
                ROS_ERROR("Failed to contact gripper");
            }

            robCmd.request.config_name = "graspMedium";
            ROS_INFO_STREAM("Moving to " << robCmd.request.config_name);
            if(!robotClient.call(robCmd))
            {
                ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                return -1;
            }

            //Close the gripper:
            ROS_INFO("Closing the gripper");
            gripper.request.state = 1.0;
            if(!urIoClient.call(gripper))
            {
                ROS_ERROR("Failed to contact gripper");
            }

            //Ask robot to go to preSmall
            robCmd.request.config_name = "preGraspMedium";
            ROS_INFO_STREAM("Moving to " << robCmd.request.config_name);
            if(!robotClient.call(robCmd))
            {
                ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                return -1;
            }

            //Ask vision system to validate brick:
            visCmd.request.color = "red";
            if(!visClient.call(visCmd))
            {
                ROS_ERROR("Vision service call failed!");
                return -1;
            }

            if(visCmd.response.result == BRICK_MATCH)
            {
                //Ask robot to go to currentBoxString
                robCmd.request.config_name = currentBoxString;
                if(!robotClient.call(robCmd))
                {
                    ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                    return -1;
                }

                //Ask vision system to validate brick:
                if(!visClient.call(visCmd))
                {
                    ROS_ERROR("Vision service call failed!");
                    return -1;
                }

                //Ask robot to open gripper

                if(visCmd.response.result == BRICK_MATCH)
                {
                    currentOrderContents[RED_BRICKS]--; //One less brick to pack :)

                    //Open the gripper:
                    ROS_INFO("Opening the gripper");
                    gripper.request.state = 0.0;
                    if(!urIoClient.call(gripper))
                    {
                        ROS_ERROR("Failed to contact gripper");
                    }
                }
                else
                {
                    ROS_ERROR("No brick to drop!");

                    //Ask robot to go to aboveDiscard
                    robCmd.request.config_name = "aboveDiscard";
                    if(!robotClient.call(robCmd))
                    {
                        ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                        return -1;
                    }

                    //Open the gripper:
                    ROS_INFO("Opening the gripper");
                    gripper.request.state = 0.0;
                    if(!urIoClient.call(gripper))
                    {
                        ROS_ERROR("Failed to contact gripper");
                    }
                } 
            }
            else
            {
                //Ask robot to go to aboveDiscard
                robCmd.request.config_name = "aboveDiscard";
                if(!robotClient.call(robCmd))
                {
                    ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                    return -1;
                }

                //Open the gripper:
                ROS_INFO("Opening the gripper");
                gripper.request.state = 0.0;
                if(!urIoClient.call(gripper))
                {
                    ROS_ERROR("Failed to contact gripper");
                }
            }
        }

        ROS_INFO_STREAM("Packing" << currentOrderContents[YELLOW_BRICKS] << "yellow bricks");
        while(currentOrderContents[YELLOW_BRICKS] > 0) //Pack all the yellow bricks
        {
            //Ask robot to go to preSmall, then graspLarge (auto-grasps)
            robCmd.request.config_name = "preGraspLarge";
            if(!robotClient.call(robCmd))
            {
                ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                return -1;
            }

            //Open the gripper:
            ROS_INFO("Opening the gripper");
            gripper.request.state = 0.0;
            if(!urIoClient.call(gripper))
            {
                ROS_ERROR("Failed to contact gripper");
            }

            robCmd.request.config_name = "graspLarge";
            if(!robotClient.call(robCmd))
            {
                ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                return -1;
            }

            //Close the gripper:
            ROS_INFO("Closing the gripper");
            gripper.request.state = 1.0;
            if(!urIoClient.call(gripper))
            {
                ROS_ERROR("Failed to contact gripper");
            }

            //Ask robot to go to preSmall
            robCmd.request.config_name = "preGraspLarge";
            if(!robotClient.call(robCmd))
            {
                ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                return -1;
            }

            //Ask vision system to validate brick:
            visCmd.request.color = "yellow";

            if(!visClient.call(visCmd))
            {
                ROS_ERROR("Vision service call failed!");
                return -1;
            }

            if(visCmd.response.result == BRICK_MATCH)
            {
                //Ask robot to go to currentBoxString
                robCmd.request.config_name = currentBoxString;
                if(!robotClient.call(robCmd))
                {
                    ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                    return -1;
                }

                //Ask vision system to validate brick:
                if(!visClient.call(visCmd))
                {
                    ROS_ERROR("Vision service call failed!");
                    return -1;
                }

                //Ask robot to open gripper

                if(visCmd.response.result == BRICK_MATCH)
                {
                    currentOrderContents[YELLOW_BRICKS]--; //One less brick to pack :)

                    //Open the gripper:
                    ROS_INFO("Opening the gripper");
                    gripper.request.state = 0.0;
                    if(!urIoClient.call(gripper))
                    {
                        ROS_ERROR("Failed to contact gripper");
                    }
                }
                else
                {
                    ROS_ERROR("No brick to drop!");
                    //Ask robot to go to aboveDiscard
                    robCmd.request.config_name = "aboveDiscard";
                    if(!robotClient.call(robCmd))
                    {
                        ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                        return -1;
                    }

                    //Open the gripper:
                    ROS_INFO("Opening the gripper");
                    gripper.request.state = 0.0;
                    if(!urIoClient.call(gripper))
                    {
                        ROS_ERROR("Failed to contact gripper");
                    }
                }
            }
            else
            {
                //Ask robot to go to aboveDiscard
                robCmd.request.config_name = "aboveDiscard";
                if(!robotClient.call(robCmd))
                {
                    ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                    return -1;
                }

                //Open the gripper:
                ROS_INFO("Opening the gripper");
                gripper.request.state = 0.0;
                if(!urIoClient.call(gripper))
                {
                    ROS_ERROR("Failed to contact gripper");
                }
            }
        }

        boxContainsOrder[currentBox] = true;

        //Now the order should be packed, notify MES-node:
        ROS_INFO_STREAM("Deleting order with id " << getOrder.response.id);
        delOrder.request.id = getOrder.response.id;
        delOrder.request.ticket = getOrder.response.ticket;

        if(!mesDelClient.call(delOrder))
        {
            ROS_INFO("Failed to delete order!");
            return -1;
        }

        ROS_INFO("Deleted order!");
        //Service-call to MES-node

        //Call MiR
    }

    return 0;
}

void feederRefill(const std_msgs::Empty::ConstPtr& msg)
{
    feederEstimates[0] = FEEDER_MAX;
    feederEstimates[1] = FEEDER_MAX;
    feederEstimates[2] = FEEDER_MAX;
}