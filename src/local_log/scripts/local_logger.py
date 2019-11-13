#!/usr/bin/env python
import mysql.connector
from local_log.msg import system_log_msg
import rospy

mydb = mysql.connector.connect(
    host="localhost", user="logger", passwd="rsd2019", database="system_log")

mycursor = mydb.cursor()

sql_cmd = "INSERT INTO log (type, node_id, description) VALUES (%s, %s, %s)"


def post_log(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s, %s, %s",
                  data.type, data.node_id, data.desc)
    # LOG_type = data.type.replace(";", ".")
    # LOG_node_id = data.node_id.replace(";", ".")
    # LOG_desc = data.desc.replace(";", ".")

    LOG_type = data.type
    LOG_node_id = data.node_id
    LOG_desc = data.desc

    val = (LOG_type, LOG_node_id, LOG_desc)

    mycursor.execute(sql_cmd, val)

    mydb.commit()


def log_listener():
    rospy.init_node('system_logging', anonymous=True)

    rospy.Subscriber('system_log', system_log_msg, post_log)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    log_listener()
