#!/usr/bin/env python

import sys
import rospy
from mes_ordering.srv import *


def get_order():
    rospy.wait_for_service('MES_GetOrder')
    try:
        getOrder = rospy.ServiceProxy('MES_GetOrder', GetOrder_srv)
        resp1 = getOrder(1)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def delete_order(id, ticket):
    rospy.wait_for_service('MES_DeleteOrder')
    try:
        deleteOrder = rospy.ServiceProxy('MES_DeleteOrder', DeleteOrder_srv)
        resp1 = deleteOrder(id, ticket)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def usage():
    return "%s [x y]" % sys.argv[0]


if __name__ == "__main__":
    print("Requested order")
    order = get_order()
    print("Id:", order.id, "Ticket:", order.ticket)
    print("blue:", order.blue, "red", order.red, "yellow", order.yellow)
    response = delete_order(order.id, order.ticket)
    print(response.msg)
    print("Done")
