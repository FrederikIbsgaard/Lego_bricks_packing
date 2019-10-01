#!/usr/bin/python3
import requests
import json

# THIS IS A COMMENT MADE BY SUNE


def get_order_ids():
    # Get orders and create a list of id's and there status
    # TODO: Find a way to detect the amount of order
    response = requests.get(HOST + '/orders')

    if response.status_code == 200:
        # Convert response to json/dictionary
        data = json.loads(response.text)

        amountOfOrders = len(data['orders'])
        # Creates a list containing 5 lists, each of 8 items, all set to 0
        # w, h = 8, 5
        # Matrix = [[0 for x in range(w)] for y in range(h)]
        ids_status = [[0 for x in range(2)] for y in range(amountOfOrders)]

        for i in range(amountOfOrders):
            ids_status[i][0] = data['orders'][i]['id']
            ids_status[i][1] = data['orders'][i]['status']

        return [ids_status, data]


def choose_order(ids_status):
    # Check order status and choose one
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
                                 json={'cell_id': 8,
                                       'event': 'Order_Start',
                                       'comment': str(id)})
        # Convert response to json/dictionary
        entry = json.loads(response.text)
        entry['log_entry']
        return data['ticket']


def delete_order(id, ticket):
    # Delete the order with the given ticket
    response = requests.delete(HOST + '/orders/' + str(id) + '/' + str(ticket))

    if response.status_code == 204:
        print('Order deleted')
        response = requests.post(
            HOST + '/log',
            json={'cell_id': 8,
                  'event': 'Order_Done',
                  'comment': str(id)})
        # Convert response to json/dictionary
        entry = json.loads(response.text)
        entry['log_entry']
    elif response.status_code == 400:
        # Convert response to json/dictionary
        data = json.loads(response.text)
        print("Deleting order:", data['message'])


# Main
HOST = 'http://127.0.0.1:5000'
response = requests.get(HOST + '/orders')
# Plain text file
# dataText = response.text
# Accessed as Dictionary
# data = json.loads(dataText)
data = json.loads(response.text)

print(response.status_code)
print(data.keys())
print(len(data['orders']))
# msg = data['orders']
# print(msg)

# Order 0
# print('Order: 0')
# print(data['orders'][0])
# print('id:', data['orders'][0]['id'])
# print('status:', data['orders'][0]['status'])
# print('blue:', data['orders'][0]['blue'])
# print('red:', data['orders'][0]['red'])
# print('yellow:', data['orders'][0]['yellow'])

# response = requests.put('http://127.0.0.1:5000/orders/1')

# data = json.loads(response.text)

# print(data)
# print(response.status_code)

if True:
    print("-------------------------------")
    ids_data = get_order_ids()
    orders_data = ids_data[1]['orders']
    print("Available id's:", ids_data[0])
    id = choose_order(ids_data[0])
    print("Chosen id:", id)
    ticket = reserve_order(id)
    print("Ticket:", ticket)
    if ticket is not None:
        for i in range(len(orders_data)):
            if orders_data[i]['id'] == id:
                print("Order:", id, "----------------")
                print("blue:", orders_data[i]['blue'])
                print("red:", orders_data[i]['red'])
                print("yellow:", orders_data[i]['yellow'])
        delete_order(id, ticket)
    print("-------------------------------")
    # response = requests.post(HOST + '/log', json={'cell_id':8, 'event':'PML_Idle', 'comment':'testing log'})
    # data = json.loads(response.text)
    # print(response.status_code)
    # print(data)
