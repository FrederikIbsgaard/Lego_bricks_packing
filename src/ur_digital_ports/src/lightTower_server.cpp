#include <iostream>
#include <vector>
#include <unistd.h>
#include <rtde_io_interface.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "ur_digital_ports/digitalOut_srv.h"
#include "state.h"

#define RED_PORT 0
#define AMBER_PORT 1
#define GREEN_PORT 2

//using namespace ur_rtde;
unsigned int redAction = 0, amberAction = 0, greenAction = 0, bottonAction = 0;
ur_rtde::RTDEIOInterface rtde_io("192.168.0.12");
// Reset everything at start
//void pmlState(const std_msgs::String::ConstPtr& msg){
void pmlState(const std_msgs::Int8::ConstPtr& msg){
	int msgs = msg->data;

	redAction = 0;
	amberAction = 0;
	greenAction = 0;

	if((msgs == ST_ABORTING) or (msgs == ST_ABORTED) or (msgs == ST_CLEARING)){
		//Red light flashing
		redAction = 2;
	}
	else if((msgs == ST_STOPPING) or (msgs == ST_STOPPED)){
		//Red light on
		redAction = 1;
	}
	else if(msgs == ST_RESETTING){
		//Amber light flashing
		amberAction = 2;
	}
	else if(msgs == ST_IDLE){
		//Green light flashing
		greenAction = 2;
	}
	else if((msgs == ST_STARTING) or (msgs == ST_EXECUTE) or (msgs == ST_UNHOLDING) or (msgs == ST_UNSUPENDING)){
		//Green light on
		greenAction = 1;
	}
	else if((msgs == ST_HOLDING) or (msgs == ST_HELD)){
		//Amber & Green lights flashing
		amberAction = 2;
		greenAction = 2;
	}
	else if((msgs == ST_SUSPENDING) or (msgs == ST_SUSPENDED)){
		//Amber light on
		amberAction = 1;
	}
	else{
		redAction = 0;
		amberAction = 0;
		greenAction = 0;
	}
	ROS_INFO("New PackML state: %d", msgs);
}

void lightOn(int port, int state){
	ROS_INFO("Port: %d, State: %d", port, state);
	// ur_rtde::RTDEIOInterface rtde_io("192.168.42.12");
	rtde_io.setStandardDigitalOut(port, state);
}

bool portControl(ur_digital_ports::digitalOut_srv::Request  &req,
         ur_digital_ports::digitalOut_srv::Response &res)
{
	// The constructor simply takes the IP address of the Robot
	ROS_INFO("request: port=%ld, state=%ld", (long int)req.port, (long int)req.state);
	res.status = rtde_io.setStandardDigitalOut(req.port, req.state);
	ROS_INFO("sending back response: [%ld]", (long int)res.status);
  return true;
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

void bottonLight(const std_msgs::Int8::ConstPtr& msg){
	int msgs = msg->data;
	if (msgs == 1){
		bottonAction = 1;
	}
	else if (msgs == 0){
		bottonAction = 0;
	}
	else{
		bottonAction = 0;
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "lightTower_server");
	ros::NodeHandle n;

	ros::Subscriber subButton = n.subscribe("set_botton_light", 10, bottonLight);
	ros::Subscriber sub = n.subscribe("packml_state", 10, pmlState);
	ros::ServiceServer service = n.advertiseService("digital_output", portControl);
	ROS_INFO("Ready to use digital output.");
	//ros::spin();
	ros::spinOnce();

	int i=0;
	bool redCurState = false, amberCurState = false, greenCurState = false, bottonCurState = false;

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

		if (bottonAction == 1){
			bottonCurState != bottonCurState;
		}
		else{
			bottonCurState = false;
		}

		lightOn(5, bottonCurState);

		ROS_INFO("i = %d", i);
		i += 1;
		usleep(600000);
	}

  return 0;
}
