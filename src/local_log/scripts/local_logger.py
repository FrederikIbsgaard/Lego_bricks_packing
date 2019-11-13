#!/usr/bin/env python
import mysql.connector
import rospy
from std_msgs.msg import String
from local_log import local_logger

mydb = mysql.connector.connect(
    host="localhost", user="logger", passwd="rsd2019", database="system_log")

print(mydb)

mycursor = mydb.cursor()

sql_cmd = "INSERT INTO log (type, node_id, description) VALUES (%s, %s, %s)"
val = ("Event", "TEST_ID", "This is a test")

mycursor.execute(sql_cmd, val)

mydb.commit()

print(mycursor.rowcount, "record inserted.")
for x in mycursor:
    print(x)

def log_listener():
	rospy.init_node

if __name__ == '__main__':
	log_listener()