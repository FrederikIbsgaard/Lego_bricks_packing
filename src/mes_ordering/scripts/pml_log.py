#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ordering_client import post_pml
from local_log.msg import system_log_msg

pub = rospy.Publisher('system_log', system_log_msg, queue_size=10)


def callback(data):
    rospy.loginfo(data.data)

    msgs = data.data

    pub.publish("INFO", "pml_logging", msgs)
    post_pml(msgs)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pml_logging', anonymous=True)

    rospy.Subscriber("packml_state", String, callback)

    print("MES PackML Ready")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
