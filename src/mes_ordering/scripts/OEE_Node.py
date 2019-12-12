#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from local_log.msg import system_log_msg
import time

pub_log = rospy.Publisher('system_log', system_log_msg, queue_size=10)
pub_oee = rospy.Publisher('oee', String, queue_size=10)
pub_avail = rospy.Publisher('oeeAvail', String, queue_size=10)
pub_perf = rospy.Publisher('oeePerf', String, queue_size=10)
pub_qual = rospy.Publisher('oeeQual', String, queue_size=10)

time0 = time.time() - time.time()


#
#
#
# MAKE A CLASS
#
#
#


class Oee_class:
    def reset_timers():
        x1.downTime_start = time0
        x1.downTime_stop = time0
        x1.runTime_start = time0
        x1.runTime_stop = time0

    runTime_start = time0
    runTime_stop = time0
    downTime_start = time0
    downTime_stop = time0
    downTime = time0
    runTime = time0
    GoodCount = 0
    TotalCount = 0
    TargetOrders = 0
    TotalOrders = 0
    downTime_flag = 0


x1 = Oee_class()


def callback(data):
    rospy.loginfo(data.data)

    msgs = data.data

    if msgs == 'runTime_start':
        if x1.downTime_flag is not 0:
            # Stop down time
            x1.downTime_stop = time.time()
            # Calculate down time and add to list
            x1.downTime += (x1.downTime_stop - x1.downTime_start)
        # Start run time
        x1.runTime_start = time.time()
    elif (msgs == 'downTime_start') and (x1.downTime_flag is not 1):
        x1.downTime_flag = 1
        # Stop run time
        x1.runTime_stop = time.time()
        # Calculate run time
        x1.runTime += x1.runTime_stop - x1.runTime_start
        # Start down time
        x1.downTime_start = time.time()
    elif (msgs == 'downTime_stop') and (x1.downTime_flag is not 0):
        x1.downTime_flag = 0
        # Stop down time
        x1.downTime_stop = time.time()
        # Calculate down time and add to list
        x1.downTime += (x1.downTime_stop - x1.downTime_start)
        # Start run time
        x1.runTime_start = time.time()
    elif msgs == 'orderTaken':
        # Update total # of taken orders
        x1.TargetOrders += 1
    elif 'done' in msgs:
        # Stop run time
        x1.runTime_stop = time.time()
        # Calculate run time
        x1.runTime += x1.runTime_stop - x1.runTime_start

        # done,# of orders completed,
        # total # of brick in orders,
        # total # of bricks touched
        msgData = str(msgs).split(",")

        # print msgData[1], msgData[2], msgData[3]
        # print int(msgData[1]), int(msgData[2]), int(msgData[3])

        # Calculate the availablity
        Availability = calc_Availability(x1.runTime, x1.downTime)

        # Update the total # of orders completed
        x1.TotalOrders += int(msgData[1])
        # Calculate the performance
        Performance = calc_Performance(
            float(x1.TotalOrders), float(x1.TargetOrders))

        # Update total # of packed bricks
        x1.GoodCount += int(msgData[2])
        # Update total # of touched bricks
        x1.TotalCount += int(msgData[3])
        # Calculate the quality
        Quality = calc_Quality(float(x1.GoodCount), float(x1.TotalCount))

        # Calculate the OEE
        OEE = Availability * Performance * Quality

        # Publish information
        pub_avail.publish(str(Availability))
        pub_perf.publish(str(Performance))
        pub_qual.publish(str(Quality))
        pub_oee.publish(str(OEE))
        pub_log.publish("INFO", "OEE_Calc", "Avail: " + str(Availability) +
                        " Perf: " + str(Performance) +
                        " Qual: " + str(Quality) + " OEE: " + str(OEE))
        x1.downTime_flag = 1
        x1.downTime_start


def calc_Availability(RunTime, DownTime):
    SetupTime = 0
    TotalTime = RunTime + SetupTime + DownTime
    return RunTime / TotalTime


def calc_Performance(TotalCount, TargetCount):
    # Completed order/Taken orders
    return TotalCount / TargetCount


def calc_Quality(GoodCount, TotalCount):
    # Amount of bricks in the 4 orders
    # / Amount of bricks taken during the 4 orders
    return GoodCount / TotalCount


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('OEE_Calc', anonymous=True)

    rospy.Subscriber("oee_calculator", String, callback)

    print("OEE Calculator Ready")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
