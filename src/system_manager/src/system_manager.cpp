#include <iostream>
#include "ros/ros.h"

//Services:
#include "robot_control/goto_config.h"
#include "vision/"

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
        currentOrderContents[0] = 3;
        currentOrderContents[1] = 2;
        currentOrderContents[2] = 3;

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

        while(currentOrderContents[BLUE_BRICKS] > 0) //Pack all the blue bricks
        {
            //Ask robot to go to preSmall, then graspSmall (auto-grasps)

            //Ask vision system to validate brick:
            visCmd.request.color = "blue";

            if(!visClient.call(visCmd))
            {
                ROS_ERROR("Vision service call failed!");
                return -1;
            }

            //Ask robot to go to preSmall

            if(visCmd.response.result == BRICK_MATCH)
            {
                //Ask robot to go to currentBoxString
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
                    ROS_ERROR("No brick to drop!"); 
            }
            else
            {
                //Ask robot to go to aboveDiscard
                //Ask robot to open gripper
            }
        }

        while(currentOrderContents[RED_BRICKS] > 0) //Pack all the red bricks
        {
            //Ask robot to go to preMedium, then graspMedium, then grasp

            //Ask vision system to validate brick

            //Ask robot to go to preMedium

            if(true /*brick is validated*/)
            {
                //Ask robot to go to currentBoxString
                //Ask vision system to validate brick
                //Ask robot to open gripper

                if(true /*brick is validated*/)
                    currentOrderContents[RED_BRICKS]--; //One less brick to pack :)
                else
                    ROS_ERROR("No brick to drop!"); 
            }
            else
            {
                //Ask robot to go to aboveDiscard
                //Ask robot to open gripper
            }
        }

        while(currentOrderContents[YELLOW_BRICKS] > 0) //Pack all the yellow bricks
        {
            //Ask robot to go to preLarge, then graspLarge, then grasp

            //Ask vision system to validate brick

            //Ask robot to go to preLarge

            if(true /*brick is validated*/)
            {
                //Ask robot to go to currentBoxString
                //Ask vision system to validate brick
                //Ask robot to open gripper

                if(true /*brick is validated*/)
                    currentOrderContents[YELLOW_BRICKS]--; //One less brick to pack :)
                else
                    ROS_ERROR("No brick to drop!"); 
            }
            else
            {
                //Ask robot to go to aboveDiscard
                //Ask robot to open gripper
            }
            
        }

        //Now the order should be packed, notify MES-node:
        //Service-call to MES-node

        //Call MiR
            
    }

    return 0;
}