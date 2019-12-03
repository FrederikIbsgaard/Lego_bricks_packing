#include <iostream>
#include <vector>
#include <unistd.h>
#include <rtde_io_interface.h>
#include "ros/ros.h"
#include "ur_digital_ports/digitalOut_srv.h"

//using namespace ur_rtde;
ur_rtde::RTDEIOInterface rtde_io("192.168.0.12");

bool toolOut(ur_digital_ports::digitalOut_srv::Request  &req,
         ur_digital_ports::digitalOut_srv::Response &res)
{
	// The constructor simply takes the IP address of the Robot
	ROS_INFO("request: port=%ld, state=%ld", (long int)req.port, (long int)req.state);
	res.status = rtde_io.setStandardDigitalOut(req.port, req.state);
	ROS_INFO("sending back response: [%ld]", (long int)res.status);
  return true;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "toolControl_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("ur_tool_out", toolOut);
	ROS_INFO("Ready to use digital tool output.");
	ros::spin();

  return 0;
}
