#include <iostream>
#include <chrono>
#include <mutex>
#include "ros/ros.h"

#include "state.h"
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <ur_dashboard_msgs/Load.h> //For loading UR programs
#include <std_srvs/Trigger.h> //For play, pause and stop programs
#include <ur_dashboard_msgs/GetProgramState.h>
#include "vision/check_brick.h"
#include "ur_digital_ports/digitalOut_srv.h"

#define BLUE_BRICKS   0
#define RED_BRICKS    1
#define YELLOW_BRICKS 2

#define BRICK_NOMATCH 0
#define BRICK_MATCH   1

using namespace std;

bool loadAndRunUrProgram(string filename);
ros::ServiceClient robotLoadProgram;
ur_dashboard_msgs::Load robotLoadSrv;

ros::ServiceClient robotPlay;
std_srvs::Trigger robotPlaySrv;

ros::ServiceClient robotGetProgState;
ur_dashboard_msgs::GetProgramState robotGetProgStateSrv;

ros::ServiceClient urIoClient;
ur_digital_ports::digitalOut_srv gripper;

//Vision:
ros::ServiceClient visClient;
vision::check_brick visCmd;

int discardCount[3];


int main(int argc, char** argv)
{
    ros::init(argc, argv, "discard_test");
    ros::NodeHandle n;

    robotLoadProgram = n.serviceClient<ur_dashboard_msgs::Load>("/ur_hardware_interface/dashboard/load_program");

    robotPlay = n.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/play");

    robotGetProgState = n.serviceClient<ur_dashboard_msgs::GetProgramState>("/ur_hardware_interface/dashboard/program_state");

    visClient = n.serviceClient<vision::check_brick>("/check_brick");

    //Gripper interface:
    //IO message for opening/closing gripper:
    gripper.request.port = 4;
    gripper.request.state = 0.0;

    urIoClient = n.serviceClient<ur_digital_ports::digitalOut_srv>("/digital_output");

    //Yellow:
    for(int i = 0; i < 32; i++)
    {
        ROS_INFO_STREAM("Yellow no: " << i);
        loadAndRunUrProgram("rsd_pick_big.urp");
        visCmd.request.color = "yellow";
        if(!visClient.call(visCmd))
        {
            ROS_ERROR("Vision service call failed!");
            throw("Vision service call failed!");
            return -1;
        }

        if(BRICK_MATCH)
        {
            loadAndRunUrProgram("rsd_box_a.urp");
            if(!urIoClient.call(gripper))
            {
                throw("Gripper error!");
                return -1;
            }
        }
        else
        {
            loadAndRunUrProgram("rsd_discard.urp");
            discardCount[YELLOW_BRICKS]++;
        }      
    }
    ROS_INFO_STREAM("\t yellow: " << discardCount[YELLOW_BRICKS]);

    //Red:
    for(int i = 0; i < 32; i++)
    {
        ROS_INFO_STREAM("Red no: " << i);
        loadAndRunUrProgram("rsd_pick_medium.urp");
        visCmd.request.color = "red";
        if(!visClient.call(visCmd))
        {
            ROS_ERROR("Vision service call failed!");
            throw("Vision service call failed!");
            return -1;
        }

        if(BRICK_MATCH)
        {
            loadAndRunUrProgram("rsd_box_b.urp");
            if(!urIoClient.call(gripper))
            {
                throw("Gripper error!");
                return -1;
            }
        }
        else
        {
            loadAndRunUrProgram("rsd_discard.urp");
            discardCount[RED_BRICKS]++;
        }
    }
    ROS_INFO_STREAM("\t red   : " << discardCount[RED_BRICKS]);

    //Blue:
    for(int i = 0; i < 32; i++)
    {
        ROS_INFO_STREAM("Blue no: " << i);
        loadAndRunUrProgram("rsd_pick_small.urp");
        visCmd.request.color = "blue";
        if(!visClient.call(visCmd))
        {
            ROS_ERROR("Vision service call failed!");
            throw("Vision service call failed!");
            return -1;
        }

        if(BRICK_MATCH)
        {
            loadAndRunUrProgram("rsd_box_c.urp");
            if(!urIoClient.call(gripper))
            {
                throw("Gripper error!");
                return -1;
            }
        }
        else
        {
            loadAndRunUrProgram("rsd_discard.urp");
            discardCount[BLUE_BRICKS]++;
        }      
    }

    ROS_INFO_STREAM("\t blue  : " << discardCount[BLUE_BRICKS]);

    return 0;
}

bool loadAndRunUrProgram(string filename)
{
    robotLoadSrv.request.filename = filename;

    if(!robotLoadProgram.call(robotLoadSrv))
    {
        ROS_ERROR_STREAM("Failed to load UR program " << robotLoadSrv.request.filename);
        return false;
    }

    if(!robotPlay.call(robotPlaySrv))
    {
        ROS_ERROR_STREAM("Failed to execute UR program " << robotLoadSrv.request.filename);
        throw("Failed to execute UR program!");
    }

    ros::Duration(1.0).sleep();

    do
    {
        if(!robotGetProgState.call(robotGetProgStateSrv))
        {
            ROS_ERROR("Failed to get robot program state!");
            return false;
        }
        ros::Duration(0.5).sleep();
    } while (robotGetProgStateSrv.response.state.state == robotGetProgStateSrv.response.state.PLAYING);
    
    return true;
}