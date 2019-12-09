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
        x1.downTime_Sum = [time0]
        x1.runTime_start = time0
        x1.runTime_stop = time0
        x1.runTime_Sum = [time0]

    runTime_Sum = [time0]
    downTime_Sum = [time0]
    runTime_start = time0
    runTime_stop = time0
    downTime_start = time0
    downTime_stop = time0


x1 = Oee_class()


def callback(data):
    rospy.loginfo(data.data)

    msgs = data.data

    if msgs == 'runTime_start':
        x1.runTime_start = time.time()
    elif msgs == 'runTime_stop':
        x1.runTime_stop = time.time()
        x1.runTime_Sum.append(x1.runTime_stop - x1.runTime_start)
    elif msgs == 'downTime_start':
        x1.downTime_start = time.time()
    elif msgs == 'downTime_stop':
        x1.downTime_stop = time.time()
        x1.downTime_Sum.append(x1.downTime_stop - x1.downTime_start)
    elif 'done' in msgs:
        # done,# of orders completed,
        # total # of brick in orders,
        # total # of bricks touched
        msgData = str(msgs).split(",")
        print msgData[1], msgData[2], msgData[3]
        print int(msgData[1]), int(msgData[2]), int(msgData[3])
        Availability = calc_Availability(
            sum(x1.runTime_Sum), sum(x1.downTime_Sum))
        Performance = calc_Performance(float(msgData[1]))
        Quality = calc_Quality(float(msgData[2]), float(msgData[3]))
        print Quality
        OEE = Availability * Performance * Quality

        pub_avail.publish(Availability)
        pub_perf.publish(Performance)
        pub_qual.publish(Quality)
        pub_oee.publish(OEE)
        pub_log.publish("INFO", "OEE_Calc", "Avail: " + str(Availability) +
                        " Perf: " + str(Performance) +
                        " Qual: " + str(Quality) + " OEE: " + str(OEE))
        x1.reset_timers


def calc_Availability(RunTime, DownTime):
    SetupTime = 0
    TotalTime = RunTime + SetupTime + DownTime
    return RunTime / TotalTime


def calc_Performance(TotalCount):
    # Completed order/Taken orders
    TargetCount = 4
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
