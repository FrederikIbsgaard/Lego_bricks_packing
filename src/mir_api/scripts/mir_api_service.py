#!/usr/bin/env python

from mir_api.srv import mir_api_action, mir_api_actionResponse
import requests
import time
import rospy
import numpy as np
import json

language = {'Accept-Language': "en_US"}


class RestMiR():
    def __init__(self):
        self.authorization = {
            'Authorization': "Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="}
        self.HOST = 'http://10.10.19.40/api/v2.0.0/'
    #We need to put the name of our mission
    def get_mission(self, mission_name="MISSION_NAME"):
        response = requests.get(self.HOST + 'missions', headers=self.authorization)
        mission = ""
        if response.status_code != 200:
            print(response.status_code)
        for counter in response.json():
            if counter["name"] == mission_name:
                mission = counter
                print(mission)
        return mission

    def add_mission_to_queue(self, mission):
        data = {"filters" : [{"fieldname": "state", "operator": "IN", "value": ["Pending", "Executing"]}]}
        r = requests.post(self.HOST + 'mission_queue/search', json=data, headers=self.authorization)
        data = {"mission_id": str(mission["guid"])}
        if  len(r.json()) > 0:
            for i in range(0, len(r.json())):
                #print(data['mission_id'])
                _id = requests.get(self.HOST + 'mission_queue/'+str(r.json()[i]['id']), headers=self.authorization)
                #print(_id.json()['mission_id'])
                if data['mission_id'] == _id.json()['mission_id']:
                    if r.json()[i]['state'] == 'Pending' or  r.json()[i]['state'] == 'Executing':
                        rospy.loginfo("ERROR Mission Already Pending or Executing")
                        return 0
        response = requests.post(self.HOST + "mission_queue", json=data, headers=self.authorization)
        if response.status_code != 201:
            print("ERROR" + str(response.status_code))
            return False
        return True

    def is_mission_executing(self, mission):
        data = {"filters" : [{"fieldname": "state", "operator": "IN", "value": ["Pending", "Executing"]}]}
        r = requests.post(self.HOST + 'mission_queue/search', json=data, headers=self.authorization)
        data = {"mission_id": str(mission["guid"])}
        if  len(r.json()) > 0:
            for i in range(0, len(r.json())):
                #print(data['mission_id'])
                _id = requests.get(self.HOST + 'mission_queue/'+str(r.json()[i]['id']), headers=self.authorization)
                #print(_id.json()['mission_id'])
                if data['mission_id'] == _id.json()['mission_id']:
                    if r.json()[i]['state'] == 'Executing':
                        return True
        return False

    #In the mission we will have to set coils (plc registers) in order to get information if robot has docked and etc.
    def read_register(self, register_id):
        response = requests.get(self.HOST + 'registers/' + str(register_id), headers=self.authorization)
        register_value = 0
        if response.status_code != 200:
            print(response.status_code)
            print(response.text)

        if (response.json()['id'] == register_id):
            register_value = response.json()['value']
        return register_value

    #As above, we can set the register when loading on the robot is ready to move to main table
    def write_register(self, register_id, value):
        data = {"value": value}
        response = requests.put(self.HOST + 'registers/' + str(register_id), json = data, headers=self.authorization)

        if response.status_code != 200:
            print(response.status_code)
        return 0



robot = RestMiR()
guid = robot.get_mission("GoToGr8")
#---------------------------------------------------------------------------------------

def handle_GotoGr8(req):
    if req.action == 1:
        #guid = robot.get_mission("GoToGr8")
        if robot.add_mission_to_queue(guid):
            return mir_api_actionResponse(1)
    elif req.action == 2:
        if robot.read_register(8) == 2:
            return mir_api_actionResponse(2)
    elif req.action == 3:
        robot.write_register(8, 1)  # MIR can go
        return mir_api_actionResponse(3)
    elif req.action == 10:
        if robot.read_register(8) > 0:
            if robot.is_mission_executing(guid):
                #rospy.loginfo("true")
                return mir_api_actionResponse(10)
            #rospy.loginfo("nor")
    elif req.action == 20:
        timer = int(robot.read_register(80))
        return mir_api_actionResponse(timer)

    return mir_api_actionResponse(0)
#---------------------------------------------------------------------------------------

def mir_api_service():
    rospy.init_node('mir_api_service')
    s = rospy.Service('mir_api/service', mir_api_action, handle_GotoGr8)
    rospy.spin()





if __name__ == "__main__":
    mir_api_service()

'''
robot = RestMiR()
guid = robot.get_mission("GoToGr5")
print(robot.add_mission_to_queue(guid))
while robot.read_register(5) != 2: # wait for MIR to arrive
    robot.read_register(5)
# add function to put boxes on MIR & flag to make sure we are done with packing
print("robot arrived")
time.sleep(2)
robot.write_register(5, 1)  # MIR can go
robot.write_register(1,1)
print("bye MIR")
'''
