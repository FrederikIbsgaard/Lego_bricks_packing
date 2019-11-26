#include <iostream>
#include "ros/ros.h"

//Services:
#include "robot_control/goto_config.h"
#include "vision/check_brick.h"

//Topics:


using namespace std;

#define BLUE_BRICKS   0
#define RED_BRICKS    1
#define YELLOW_BRICKS 2

#define BRICK_NOMATCH 0
#define BRICK_MATCH   1

//Order info:
int currentOrderContents[3]; //blue, red, yellow
bool boxContainsOrder[4] = {false, false, false, false};
int currentOrderId;
int currentOrderTicket;


int currentBox;
string currentBoxString;
bool running = true;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "system_manager");
    ros::NodeHandle n;

    //Setup service clients:
    ros::ServiceClient robotClient = n.serviceClient<robot_control::goto_config>("/go_to_config");
    robot_control::goto_config robCmd;

    //ros::ServiceClient mesGetClient = n.serviceClient<mes_ordering::GetOrder_srv>("/MES_GetOrder");
    //mes_ordering::GetOrder_srv mesCmd;

    //ros::ServiceClient mesDelClient = n.serviceClient<mes_ordering::DeleteOrder_srv>("/MES_DeleteOrder");

    ros::ServiceClient visClient = n.serviceClient<vision::check_brick>("/check_brick");
    vision::check_brick visCmd;


    //Start packing:
    ROS_INFO("System manager started!");
    ros::Duration(1).sleep();

    while(running)
    {
        ROS_INFO("Asking MES system for next order...");
        ros::Duration(1).sleep();

        //Input fake order, for now:
        currentOrderContents[0] = 2;
        currentOrderContents[1] = 5;
        currentOrderContents[2] = 5;

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

            robCmd.request.config_name = "graspSmall";
            ROS_INFO_STREAM("Moving to " << robCmd.request.config_name);
            if(!robotClient.call(robCmd))
            {
                ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                return -1;
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
            visCmd.request.color = "blue";
            if(!visClient.call(visCmd))
            {
                ROS_ERROR("Vision service call failed!");
                return -1;
            }

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

                //Ask robot to open gripper

                if(visCmd.response.result == BRICK_MATCH)
                    currentOrderContents[BLUE_BRICKS]--; //One less brick to pack :)
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

            robCmd.request.config_name = "graspMedium";
            ROS_INFO_STREAM("Moving to " << robCmd.request.config_name);
            if(!robotClient.call(robCmd))
            {
                ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                return -1;
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
                    currentOrderContents[RED_BRICKS]--; //One less brick to pack :)
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

            robCmd.request.config_name = "graspLarge";
            if(!robotClient.call(robCmd))
            {
                ROS_ERROR_STREAM("Failed to move robot to " << robCmd.request.config_name);
                return -1;
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
                    currentOrderContents[YELLOW_BRICKS]--; //One less brick to pack :)
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
            }
        }

        boxContainsOrder[currentBox] = true;


        //Now the order should be packed, notify MES-node:
        //Service-call to MES-node

        //Call MiR
            
    }

    return 0;
}