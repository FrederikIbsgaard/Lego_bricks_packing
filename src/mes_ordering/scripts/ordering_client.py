#!/usr/bin/python3
import requests
import json


def get_orders():
    # Get orders and create a list of id's and their status
    response = requests.get(HOST + '/orders')

    if response.status_code == 200:
        # Convert response to json/dictionary
        data = json.loads(response.text)

        # Check aomunt of orders
        amountOfOrders = len(data['orders'])

        if amountOfOrders is not 0:
            # Creates a list containing 5 lists, each of 8 items, all set to 0
            # w, h = 8, 5
            # Matrix = [[0 for x in range(w)] for y in range(h)]

            # Matrix     [[number of elements]   number of lists]
            ids_status = [[0 for x in range(2)] for y in range(amountOfOrders)]

            for i in range(amountOfOrders):
                ids_status[i][0] = data['orders'][i]['id']
                ids_status[i][1] = data['orders'][i]['status']

            return [ids_status, data]
        else:
            print("There are no orders")


def choose_order(ids_status):
    # Check chooses an available order and returns its id
    # TODO: Create selection method
    for i in range(len(ids_status)):
        if ids_status[i][1] == 'ready':
            return ids_status[i][0]
    print('No order ready')


def reserve_order(id):
    # Reserve order with given id
    response = requests.put(HOST + '/orders/' + str(id))

    # Convert response to json/dictionary
    data = json.loads(response.text)

    if response.status_code == 400:
        print("Reserving order:", data['message'])
    elif response.status_code == 200:
        response = requests.post(HOST + '/log',
                                 json={'cell_id': CELL_ID,
                                       'event': 'Order_Start',
                                       'comment': str(id)})
        # Convert response to json/dictionary
        entry = json.loads(response.text)
        entry['log_entry']
        return data['ticket']


def get_order_info(id, order_data):
    print("Order:", id, "----------------")
    print("blue:", order_data['blue'])
    print("red:", order_data['red'])
    print("yellow:", order_data['yellow'])


def delete_order(id, ticket):
    # Delete the order with the given ticket
    response = requests.delete(HOST + '/orders/' + str(id) + '/' + str(ticket))

    if response.status_code == 204:
        print('Order deleted')
        response = requests.post(
            HOST + '/log',
            json={'cell_id': CELL_ID,
                  'event': 'Order_Done',
                  'comment': str(id)})
        # Convert response to json/dictionary
        entry = json.loads(response.text)
        entry['log_entry']
    elif response.status_code == 400:
        # Convert response to json/dictionary
        data = json.loads(response.text)
        print("Deleting order:", data['message'])


def post_pml(PML_State):
    # Post the current PackML state to the log
    response = requests.post(
        HOST + '/log',
        json={'cell_id': CELL_ID,
              'event': PML_State,
              'comment': ""})

    if response.status_code == 201:
        print(PML_State)
        # Convert response to json/dictionary
        entry = json.loads(response.text)
        entry['log_entry']

    elif response.status_code == 400:
        # Convert response to json/dictionary
        data = json.loads(response.text)
        print("Logging PML:", data['message'])


# Global variables
HOST = 'http://127.0.0.1:5000'
CELL_ID = 8

# Main
if __name__ == "__main__":
    if True:
        print("-------------------------------")
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
                            break
                    delete_order(id, ticket)
        print("-------------------------------")
