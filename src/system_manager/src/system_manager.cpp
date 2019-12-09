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

#include <ur_msgs/SetIO.h>

//Services:
// #include "robot_control/goto_config.h"
#include <ur_dashboard_msgs/Load.h> //For loading UR programs
#include <std_srvs/Trigger.h> //For play, pause and stop programs
#include <ur_dashboard_msgs/GetProgramState.h>
#include "vision/check_brick.h"
#include "mes_ordering/DeleteOrder_srv.h"
#include "mes_ordering/GetOrder_srv.h"
#include "mir_api/mir_api_action.h"

//Topics:

using namespace std;

#define BLUE_BRICKS   0
#define RED_BRICKS    1
#define YELLOW_BRICKS 2

#define BRICK_NOMATCH 0
#define BRICK_MATCH   1

#define FEEDER_WARNING_THRESH 5

#define FEEDER_MAX 32 //Max number of bricks in any feeder

#define MIR_CALL         1
#define MIR_POLL_ARRIVED 2
#define MIR_RELEASE      3

//Order info:
int currentOrderContents[3]; //blue, red, yellow
bool boxContainsOrder[4] = {false, false, false, false};
int currentOrderId;
string currentOrderTicket;

//Estimate of feeder contents:
int feederEstimates[3] = {FEEDER_MAX, FEEDER_MAX, FEEDER_MAX};

//Runtime info:
int currentBox;
string currentBoxString;
bool running = true;
bool isPaused = false;
bool isStopped = true;

//Callbacks:
void feederRefill(const std_msgs::Empty::ConstPtr& msg);
void pauseSystem(const std_msgs::Empty::ConstPtr& msg);
void playSystem(const std_msgs::Empty::ConstPtr& msg);
void stopSystem(const std_msgs::Empty::ConstPtr& msg);

//Robotics:
bool loadAndRunUrProgram(string filename);
ros::ServiceClient robotLoadProgram;
ur_dashboard_msgs::Load robotLoadSrv;

ros::ServiceClient robotPlay;
std_srvs::Trigger robotPlaySrv;

ros::ServiceClient robotPause;
std_srvs::Trigger robotPauseSrv;

ros::ServiceClient robotStop;
std_srvs::Trigger robotStopSrv;

ros::ServiceClient robotGetProgState;
ur_dashboard_msgs::GetProgramState robotGetProgStateSrv;

ros::ServiceClient urIoClient;
ur_msgs::SetIO gripper;

ros::ServiceClient visClient;
vision::check_brick visCmd;

ros::ServiceClient mesGetClient;
mes_ordering::GetOrder_srv getOrder;

//Mutexes:
mutex feederLock;
mutex pauseLock;
mutex orderLock;
mutex stopLock;

//Functions:
void getOrderFromMes(int &orderId, int &b, int &r, int &y, string &ticket);
void pack(int color, int amount, int box);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "system_manager");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    //Setup service clients:
    //Robot:
    robotLoadProgram = n.serviceClient<ur_dashboard_msgs::Load>("/ur_hardware_interface/dashboard/load_program");

    robotPlay = n.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/play");

    robotPause = n.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/pause");

    robotStop = n.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/stop");

    robotGetProgState = n.serviceClient<ur_dashboard_msgs::GetProgramState>("/ur_hardware_interface/dashboard/program_state");

    //MES:
    mesGetClient = n.serviceClient<mes_ordering::GetOrder_srv>("/MES_GetOrder");
    // mes_ordering::GetOrder_srv mesCmd;

    mes_ordering::GetOrder_srv getOrder;

    ros::ServiceClient mesDelClient = n.serviceClient<mes_ordering::DeleteOrder_srv>("/MES_DeleteOrder");

    mes_ordering::DeleteOrder_srv delOrder;

    //Vision:
    visClient = n.serviceClient<vision::check_brick>("/check_brick");

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
    gripper.request.fun = 1;
    gripper.request.pin = 4;
    gripper.request.state = 1.0;

    urIoClient = n.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");

    //MiR interface:
    ros::ServiceClient mirClient = n.serviceClient<mir_api::mir_api_action>("/mir_api/service");
    mir_api::mir_api_action mir;

    //Sub. to system topics:
    ros::Subscriber pauseSub = n.subscribe<std_msgs::Empty>("/gui_pause", 1, pauseSystem);
    ros::Subscriber playSub = n.subscribe<std_msgs::Empty>("/gui_play", 1, playSystem);
    ros::Subscriber postSub = n.subscribe<std_msgs::Empty>("/gui_stop", 1, stopSystem);

    //Start packing:
    //Open the gripper:
    ROS_INFO("Opening the gripper");
    gripper.request.state = 0.0;
    if(!urIoClient.call(gripper))
    {
        ROS_ERROR("Failed to contact gripper");
    }
    
    ROS_INFO("System manager started!");

    stopLock.lock();
    bool stopCopy = isStopped;
    stopLock.unlock();

    do
    {
        stopLock.lock();
        stopCopy = isStopped;
        stopLock.unlock();
        ros::Duration(0.1).sleep();
    } while(stopCopy);

    while(running)
    {
        //Stop-wait-loop:
        do
        {
            stopLock.lock();
            stopCopy = isStopped;
            stopLock.unlock();
            ros::Duration(0.1).sleep();
        } while(stopCopy);

        stopLock.lock();
        stopCopy = isStopped;
        stopLock.unlock();

        if(!stopCopy)
        {
            ROS_INFO("Checking feeder status..");
            std_msgs::Empty empty;

            feederLock.lock();
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
            feederLock.unlock();
        }
        else
        {
            ROS_INFO("Stopped!");
        }
        

        stopLock.lock();
        stopCopy = isStopped;
        stopLock.unlock();

        if(!stopCopy)
        {
            ROS_INFO("Asking MES system for next order...");
            getOrderFromMes(currentOrderId, currentOrderContents[BLUE_BRICKS], currentOrderContents[RED_BRICKS], currentOrderContents[YELLOW_BRICKS], currentOrderTicket);

            ROS_INFO_STREAM("Got order no. " << currentOrderId);
            ROS_INFO_STREAM("Ticket: " << currentOrderTicket);
            ROS_INFO_STREAM("Blue: " << currentOrderContents[BLUE_BRICKS]);
            ROS_INFO_STREAM("Red: " << currentOrderContents[RED_BRICKS]);
            ROS_INFO_STREAM("Yellow: " << currentOrderContents[YELLOW_BRICKS]);

            //Find the next available box:
            currentBox = 0;
            while (boxContainsOrder[currentBox] && currentBox < 4)
                currentBox++;

            ROS_INFO_STREAM("Using box no: " << currentBox);
        }
        else
        {
            ROS_INFO("Stopped!");
        }

        ROS_INFO_STREAM("Packing in box " << currentBoxString);

        stopLock.lock();
        stopCopy = isStopped;
        stopLock.unlock();

        if(!stopCopy)
        {
            ROS_INFO("Packing blue bricks");
            pack(BLUE_BRICKS, currentOrderContents[BLUE_BRICKS], currentBox);
        }
        else
        {
            ROS_INFO("Stopped!");
        }

        stopLock.lock();
        stopCopy = isStopped;
        stopLock.unlock();

        if(!stopCopy)
        {
            ROS_INFO("Packing red bricks");
            pack(RED_BRICKS, currentOrderContents[RED_BRICKS], currentBox);
        }
        else
        {
            ROS_INFO("Stopped!");
        }
        
        stopLock.lock();
        stopCopy = isStopped;
        stopLock.unlock();

        if(!stopCopy)
        {
            ROS_INFO("Packing yellow bricks");
            pack(YELLOW_BRICKS, currentOrderContents[YELLOW_BRICKS], currentBox);
        }
        else
        {
            ROS_INFO("Stopped!");
        }
        
        boxContainsOrder[currentBox] = true;

        //Now the order should be packed, notify MES-node:
        stopLock.lock();
        stopCopy = isStopped;
        stopLock.unlock();

        if(!stopCopy)
        {
            ROS_INFO_STREAM("Deleting order with id " << getOrder.response.id);
            delOrder.request.id = currentOrderId;
            delOrder.request.ticket = currentOrderTicket;

            if(!mesDelClient.call(delOrder))
            {
                ROS_INFO("Failed to delete order!");
                return -1;
            }

            ROS_INFO("Deleted order!");
        }

        //Call MiR:
        ROS_INFO("Calling MiR!");
        mir.request.action = MIR_CALL;
        if(!mirClient.call(mir))
        {
            ROS_ERROR("Service call to mir failed");
            return -1;
        }

        if(mir.response.result > 0)
        {
            ROS_INFO("MiR is on the way!");
        }
        else
        {
            ROS_INFO("MiR is busy");
        }

        mir.request.action = MIR_POLL_ARRIVED;
        if(!mirClient.call(mir))
        {
            ROS_ERROR("Service call to mir failed");
            return -1;
        }

        while(mir.response.result == 0)
        {
            ros::Duration(1).sleep();
            if(!mirClient.call(mir))
            {
                ROS_ERROR("Service call to mir failed");
                return -1;
            }
        }

        ROS_INFO("MiR has arrived!");
        //Move packed orders onto MiR:
        ROS_INFO("Placing order on MiR!");
        loadAndRunUrProgram("rsd_mir_place.urp");

        //Take empty boxes from MiR:
        ROS_INFO("Taking empty boxes from MiR");
        loadAndRunUrProgram("rsd_mir_pickup.urp");

        mir.request.action = MIR_RELEASE;
        if(!mirClient.call(mir))
        {
            ROS_ERROR("Service call to mir failed");
            return -1;
        }

        ROS_INFO("MiR is released!");



    }

    return 0;
}

void feederRefill(const std_msgs::Empty::ConstPtr& msg)
{
    feederLock.lock();
    feederEstimates[0] = FEEDER_MAX;
    feederEstimates[1] = FEEDER_MAX;
    feederEstimates[2] = FEEDER_MAX;
    feederLock.unlock();
}

bool loadAndRunUrProgram(string filename)
{
    robotLoadSrv.request.filename = filename;

    if(!robotLoadProgram.call(robotLoadSrv))
    {
        ROS_ERROR_STREAM("Failed to load UR program " << robotLoadSrv.request.filename);
        return false;
    }

    //Check if system is paused:
    pauseLock.lock();
    bool isPausedCopy = isPaused;
    pauseLock.unlock();

    while(isPausedCopy)
    {
        pauseLock.lock();
        isPausedCopy = isPaused;
        pauseLock.unlock();
        ros::Duration(0.1).sleep();
    }

    if(!robotPlay.call(robotPlaySrv))
    {
        ROS_ERROR_STREAM("Failed to execute UR program " << robotLoadSrv.request.filename);
        return false;
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

void pauseSystem(const std_msgs::Empty::ConstPtr& msg)
{
    ROS_INFO("Pausing!");
    pauseLock.lock();
    isPaused = true;
    pauseLock.unlock();
    ROS_INFO("Paused!");
}

void playSystem(const std_msgs::Empty::ConstPtr& msg)
{
    ROS_INFO("Play!");
    pauseLock.lock();
    if(isPaused)
    {
        ROS_INFO("Robot was paused!");
        isPaused = false;
    }
    pauseLock.unlock();

    stopLock.lock();
    if(isStopped)
    {
        ROS_INFO("Robot was stopped!");
        isStopped = false;
    }
    stopLock.unlock();
}

void stopSystem(const std_msgs::Empty::ConstPtr& msg)
{
    stopLock.lock();
    isStopped = true;
    stopLock.unlock();

    orderLock.lock();
    currentOrderContents[BLUE_BRICKS] = 0;
    currentOrderContents[RED_BRICKS] = 0;
    currentOrderContents[YELLOW_BRICKS] = 0;
    currentOrderId = -1;
    currentOrderTicket = -1;
    orderLock.unlock();
}

/**
 * Functions
 **/

void getOrderFromMes(int &orderId, int &b, int &r, int &y, string &ticket)
{
    orderLock.lock();
    //Call service:
    getOrder.request.amount = 1;

    if(!mesGetClient.call(getOrder))
    {
        ROS_ERROR("MES servicer call failed!");
        orderLock.unlock();
        return;
    }

    //Update param.:
    orderId = getOrder.response.id;
    b = getOrder.response.blue;
    r = getOrder.response.red;
    y = getOrder.response.yellow;
    ticket = getOrder.response.ticket;
    orderLock.unlock();
}

void pack(int color, int amount, int box)
{
    ROS_INFO("Packing order!");
    ROS_INFO_STREAM("Color: " << color);
    ROS_INFO_STREAM("Amount: " << amount);
    ROS_INFO_STREAM("Box: " << box);

    string urPickProg;
    string colorName;

    switch(color)
    {
        case BLUE_BRICKS:
            urPickProg = "rsd_pick_small.urp";
            colorName = "blue";
            break;
        case RED_BRICKS:
            urPickProg = "rsd_pick_medium.urp";
            colorName = "red";
            break;
        case YELLOW_BRICKS:
            urPickProg = "rsd_pick_big.urp";
            colorName = "yellow";
            break;
    }

    bool stopCopy;
    while(currentOrderContents[color] > 0) //Pack all the "color" bricks
    {
        //Pickup a brick:
        if(!loadAndRunUrProgram(urPickProg))
        {
            ROS_ERROR("Robot failed!");
            return;
        }

        stopLock.lock();
        stopCopy = isStopped;
        stopLock.unlock();

        if(stopCopy)
            return;

        //Ask vision system to validate brick:
        ros::WallTime start_ = ros::WallTime::now();
        visCmd.request.color = colorName;
        if(!visClient.call(visCmd))
        {
            ROS_ERROR("Vision service call failed!");
            return;
        }
        ros::WallTime end_ = ros::WallTime::now();
        double execution_time = (end_ - start_).toNSec() * 1e-6;
        ROS_INFO_STREAM("Vision time [ms]: " << execution_time);

        ROS_INFO_STREAM("Brick check result: " << (int)visCmd.response.result);
        if(visCmd.response.result == BRICK_MATCH)
        {
            //Ask robot to go to current box:
            string boxProgram;
            switch (box)
            {
            case 0:
                boxProgram = "rsd_box_a.urp";
                break;
            case 1:
                boxProgram = "rsd_box_b.urp";
                break;
            case 2:
                boxProgram = "rsd_box_c.urp";
                break;
            case 3:
                boxProgram = "rsd_box_d.urp";
                break;
            
            default:
                ROS_ERROR("Invalid box was selected. Program error, please debug.");
                return;
            }

            stopLock.lock();
            stopCopy = isStopped;
            stopLock.unlock();

            if(stopCopy)
                return;
            
            if(!loadAndRunUrProgram(boxProgram))
            {
                ROS_ERROR("Robot failed!");
                return;
            }

            //Ask vision system to validate brick:
            if(!visClient.call(visCmd))
            {
                ROS_ERROR("Vision service call failed!");
                return;
            }

            if(visCmd.response.result == BRICK_MATCH)
            {
                currentOrderContents[color]--; //One less brick to pack :)
                //Open the gripper:
                stopLock.lock();
                stopCopy = isStopped;
                stopLock.unlock();

                if(stopCopy)
                    return;
                ROS_INFO("Opening the gripper");
                gripper.request.state = 0.0;
                if(!urIoClient.call(gripper))
                {
                    ROS_ERROR("Failed to contact gripper");
                    return;
                }
            }
            else
            {
                ROS_ERROR("No brick to drop!");

                stopLock.lock();
                stopCopy = isStopped;
                stopLock.unlock();

                if(stopCopy)
                    return;
                //Ask robot to go to aboveDiscard
                if(!loadAndRunUrProgram("rsd_discard.urp"))
                {
                    ROS_ERROR("Robot failed!");
                    return;
                }
            }
        }
        else
        {
            stopLock.lock();
            stopCopy = isStopped;
            stopLock.unlock();

            if(stopCopy)
                return;
            //Ask robot to go to aboveDiscard
            if(!loadAndRunUrProgram("rsd_discard.urp"))
            {
                ROS_ERROR("Robot failed!");
                return;
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
}