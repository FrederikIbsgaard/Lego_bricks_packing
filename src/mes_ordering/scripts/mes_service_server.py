#!/usr/bin/env python

from mes_ordering.srv import GetOrder_srv, GetOrder_srvResponse, DeleteOrder_srv, DeleteOrder_srvResponse
from ordering_client import get_orders, choose_order, reserve_order, get_order_info, delete_order
import rospy


def ordering(req):
    print("req: ", req.amount)

    ids_data = get_orders()
    if ids_data is not None:
        orders_data = ids_data[1]['orders']
        print("Available id's:", ids_data[0])

        id = choose_order(ids_data[0])
        print("Chosen id:", id)

        if id is not None:
            ticket = reserve_order(id)
            print("Ticket:", ticket)
            if ticket is not None:
                for i in range(len(orders_data)):
                    if orders_data[i]['id'] == id:
                        get_order_info(id, orders_data[i])
                        orderInfo = orders_data[i]
                        break
            return GetOrder_srvResponse(id, ticket,
                                        orderInfo['blue'],
                                        orderInfo['red'],
                                        orderInfo['yellow'])

    return GetOrder_srvResponse(0, "None", 0, 0, 0)


def deleting(req):
    print("id:", req.id, "ticket:", req.ticket)
    delete_order(req.id, req.ticket)
    return DeleteOrder_srvResponse("Order Deleted")


def start_service():
    rospy.init_node('MES_Ordering')
    rospy.Service('MES_GetOrder', GetOrder_srv, ordering)
    rospy.Service('MES_DeleteOrder', DeleteOrder_srv, deleting)
    print ("Service Ready")
    rospy.spin()


# Main
if __name__ == "__main__":
    start_service()
