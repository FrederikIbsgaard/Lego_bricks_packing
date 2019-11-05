#include <iostream>
#include <vector>
#include <unistd.h>
#include <rtde_io_interface.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ur_digital_ports/digitalOut_srv.h"

#define RED_PORT 0
#define AMBER_PORT 1
#define GREEN_PORT 3

//using namespace ur_rtde;
unsigned int redAction = 0, amberAction = 0, greenAction = 0;
ur_rtde::RTDEIOInterface rtde_io("192.168.42.12");
// Reset everything at start
void pmlState(const std_msgs::String::ConstPtr& msg){
	std::string msgs = msg->data.c_str();

	redAction = 0;
	amberAction = 0;
	greenAction = 0;

	if((msgs == "Aborting") or (msgs == "Aborted") or (msgs == "Clearing")){
		//Red light flashing
		redAction = 2;
	}
	else if((msgs == "Stopping") or (msgs == "Stopped")){
		//Red light on
		redAction = 1;
	}
	else if(msgs == "Resetting"){
		//Amber light flashing
		amberAction = 2;
	}
	else if(msgs == "Idle"){
		//Green light flashing
		greenAction = 2;
	}
	else if((msgs == "Starting") or (msgs == "Executing") or (msgs == "Unholding") or (msgs == "Unsuspending")){
		//Green light on
		greenAction = 1;
	}
	else if((msgs == "Holding") or (msgs == "Held")){
		//Amber & Green lights flashing
		amberAction = 2;
		greenAction = 2;
	}
	else if((msgs == "Suspending") or (msgs == "Suspended")){
		//Amber light on
		amberAction = 1;
	}
	else{
		redAction = 0;
		amberAction = 0;
		greenAction = 0;
	}
	ROS_INFO("New PackML state: %s", msgs.c_str());
}

void lightOn(int port, int state){
	ROS_INFO("Light. Port: %d, State: %d", port, state);
	ur_rtde::RTDEIOInterface rtde_io("192.168.42.12");
	rtde_io.setStandardDigitalOut(port, state);
}

void lightT(int redS, int amberS, int greenS){
	ROS_INFO("Red: %d, Amber: %d, Green: %d", redS, amberS, greenS);
	//ur_rtde::RTDEIOInterface rtde_io("192.168.42.12");
	rtde_io.setStandardDigitalOut(RED_PORT, redS);
	//usleep(500);
	rtde_io.setStandardDigitalOut(AMBER_PORT, amberS);
	//usleep(500);
	rtde_io.setStandardDigitalOut(GREEN_PORT, greenS);
}


int main(int argc, char **argv){
	ros::init(argc, argv, "lightTower_server");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("ur_lightTower_out", 10, pmlState);
	//ros::ServiceServer service = n.advertiseService("ur_digital_out", testFunc);
	ROS_INFO("Ready to use digital output.");
	//ros::spin();
	ros::spinOnce();

	int i=0;
	bool redCurState = false, amberCurState = false, greenCurState = false;

	while(ros::ok() and ros::master::check()){
		ros::spinOnce();
		//ROS_INFO("Red: %d, Amber: %d, Green: %d", redAction, amberAction, greenAction);
		
		//Sync up the amber and green light
		if((amberAction == 2) and (greenAction == 2)){
			amberCurState = greenCurState;
		}

		// Red Light
		if((redAction == 1) and (redCurState != true)){
			redCurState = true;
			//lightOn(RED_PORT, redCurState);
		}
		else if(redAction == 2){
			redCurState = !redCurState;
			//lightOn(RED_PORT, redCurState);
		}
		else if((redAction == 0) and (redCurState == true)){
			redCurState = false;
			//lightOn(RED_PORT, redCurState);
		}

		// Amber Light
		if((amberAction == 1) and (amberCurState != true)){
			amberCurState = true;
			//lightOn(AMBER_PORT, amberCurState);
		}
		else if(amberAction == 2){
			amberCurState = !amberCurState;
			//lightOn(AMBER_PORT, amberCurState);
		}
		else if((amberAction == 0) and (amberCurState == true)){
			amberCurState = false;
			//lightOn(AMBER_PORT, amberCurState);
		}

		// Green Light
		if((greenAction == 1) and (greenCurState != true)){
			greenCurState = true;
			//lightOn(GREEN_PORT, greenCurState);
		}
		else if(greenAction == 2){
			greenCurState = !greenCurState;
			//lightOn(GREEN_PORT, greenCurState);
		}
		else if((greenAction == 0) and (greenCurState == true)){
			greenCurState = false;
			//lightOn(GREEN_PORT, greenCurState);
		}

		lightT(redCurState, amberCurState, greenCurState);

		ROS_INFO("i = %d", i);
		i += 1;
		usleep(600000);
	}

  return 0;
}
