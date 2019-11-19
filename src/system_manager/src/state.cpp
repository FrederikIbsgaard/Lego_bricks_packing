#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"

#include <sstream>
#include "state.h"
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */


int currentState = ST_ABORTING;

int state(int currentSate, int action);

void actionCallback(const std_msgs::Int16::ConstPtr& msg)
{
   int data = msg->data;
   currentState = state(currentState,data);
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "state");
  ros::NodeHandle n;
  ros::Publisher state_pub = n.advertise<std_msgs::Int16>("current_state", 1000);
  ros::Subscriber action_sub = n.subscribe("action_state", 1000, actionCallback);

  ros::Rate loop_rate(10);
  int count = 0;
  ;
  while (ros::ok())
  {
    std_msgs::Int16 msg;
    msg.data = currentState;
    state_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

int state(int currentSate, int action)
{
  if (action == AC_ABORT)
  {
    return ST_ABORTING;
  }

  switch (currentSate)
  {
    case ST_ABORTING:
      if (action == AC_SC)
        return ST_ABORTED;
      else
        return currentSate;
    case ST_ABORTED:
      if (action == AC_CLEAR)
        return ST_CLEARING;
      else
        return currentSate;
    case ST_CLEARING:
      if (action == AC_SC)
        return ST_STOPPED;
      else
        return currentSate;
    case ST_STOPPING:
      if (action == AC_SC)
        return ST_STOPPED;
      else
        return currentSate;
    case ST_STOPPED:
      if (action == AC_RESET)
        return ST_RESETTING;
      else
        return currentSate;
    case ST_RESETTING:
      if (action == AC_STOP)
        return ST_STOPPING;
      else if(action == AC_SC)
        return ST_IDLE;
      else
        return currentSate;
    case ST_IDLE:
      if (action == AC_STOP)
        return ST_STOPPING;
      else if(action == AC_START)
        return ST_STARTING;
      else
        return currentSate;
    case ST_STARTING:
      if (action == AC_STOP)
        return ST_STOPPING;
      else if(action == AC_SC)
        return ST_EXECUTE;
      else
        return currentSate;
    case ST_EXECUTE:
      if (action == AC_STOP)
        return ST_STOPPING;
      else if(action == AC_SC)
        return ST_COMPLETING;
      else if(action == AC_HOLD)
        return ST_SUSPENDING;
      else if(action == AC_SUSPEND)
        return ST_HOLDING;
      else
        return currentSate;
    case ST_COMPLETING:
      if (action == AC_STOP)
        return ST_STOPPING;
      else if(action == AC_SC)
        return ST_COMPLETE;
      else
        return currentSate;
    case ST_COMPLETE:
      if (action == AC_STOP)
        return ST_STOPPING;
      else if(action == AC_RESEST)
        return ST_RESETTING;
      else
        return currentSate;
    case ST_HOLDING:
      if (action == AC_STOP)
        return ST_STOPPING;
      else if(action == AC_SC)
        return ST_HELD;
      else
        return currentSate;
    case ST_HELD:
      if (action == AC_STOP)
        return ST_STOPPING;
      else if(action == AC_UNHOLD)
        return ST_UNHOLDING;
      else
        return currentSate;
    case ST_UNHOLDING:
      if (action == AC_STOP)
        return ST_STOPPING;
      else if(action == AC_SC)
        return ST_EXECUTE;
      else
        return currentSate;
    case ST_SUSPENDING:
      if (action == AC_STOP)
        return ST_STOPPING;
      else if(action == AC_SC)
        return ST_SUSPENDED;
      else
        return currentSate;
    case ST_SUSPENDED:
      if (action == AC_STOP)
        return ST_STOPPING;
      else if(action == AC_UNSUSPENDED)
        return ST_UNSUPENDING;
      else
        return currentSate;
    case ST_UNSUPENDING:
      if (action == AC_STOP)
        return ST_STOPPING;
      else if(action == AC_SC)
        return ST_EXECUTE;
      else
        return currentSate;
    default:
    return currentSate;
  }
  return 0;
}
