#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ordering_client import post_pml


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    msgs = data.data

    post_pml(msgs)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pml_logging', anonymous=True)

    rospy.Subscriber("pml_log", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
