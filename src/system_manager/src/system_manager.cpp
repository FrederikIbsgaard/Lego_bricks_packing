#include <iostream>
#include "ros/ros.h"

using namespace std;

int currentOrderContents[3]; //b, r, y
bool boxContainsOrder[4] = {false, false, false, false};
int currentBox;
string currentBoxString;
bool running = true;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "system_manager");
    ros::NodeHandle n;

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

        currentBox = 0;
        while (boxContainsOrder[currentBox])
            currentBox++;

        if(currentBox < 3)
        {
            ROS_ERROR("All boxes are filled with orders!");
            return -1;
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
            ROS_ERROR("Invalid box was selected. Program error.");
            return -1;
            break;
        }

        ROS_INFO_STREAM("Packing in box " << currentBoxString);

        while(currentOrderContents[0] > 0) //Pack all the blue bricks..
        {
            //Ask robot to go to preSmall, then graspSmall, then grasp

            //Ask vision system to validate brick

            if(true /*brick is validated*/)
            {
                //Ask robot to go to preSmall, then currentBoxString
                //Ask vision system to validate brick
                //Ask robot to open gripper

                if(true /*brick is validated*/)
                    currentOrderContents[0]--; //One less brick to pack :)
                else
                    ROS_ERROR("No brick to drop!"); 
            }
            else
            {
                //Ask robot to go to aboveDiscard
                //Ask robot to open gripper
            }
            
        }
            
    }

    return 0;
}