#!/usr/bin/env python

from mir_api.srv import mir_api_action, mir_api_actionResponse
from std_msgs.msg import Int8
import requests
import time
import rospy


language = {'Accept-Language': "en_US"}


class RestMiR():
    def __init__(self):
        self.authorization = {
            'Authorization': "Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="}
        self.HOST = 'http://10.10.19.40/api/v2.0.0/'
        #rospy.init_node('mir_api_node', anonymous=True)
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
        r = requests.get(self.HOST + 'mission_queue', headers=self.authorization)
        data = {"mission_id": str(mission["guid"])}
        for i in[-1,-2,-3,-4]:
            #print(data['mission_id'])
            _id = requests.get(self.HOST + 'mission_queue/'+str(r.json()[i]['id']), headers=self.authorization)
            #print(_id.json()['mission_id'])
            if data['mission_id'] == _id.json()['mission_id']:
                if r.json()[i]['state'] == 'Pending' or  r.json()[i]['state'] == 'Executing':
                    print("ERROR" + " Already Pending or Executing")
                    return 0
        response = requests.post(self.HOST + "mission_queue", json=data, headers=self.authorization)
        if response.status_code != 201:
            print("ERROR" + str(response.status_code))
            return False
        return True

    def is_mission_executing(self, mission):
        r = requests.get(self.HOST + 'mission_queue', headers=self.authorization)
        data = {"mission_id": str(mission["guid"])}
        for i in[-1,-2,-3,-4]:
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
#---------------------------------------------------------------------------------------

class mir_pubsub():
    def __init__(self):
        self.robot = RestMiR()
        rospy.init_node('mir_api_pubSub', anonymous=True)
        self.pub = rospy.Publisher('mir_api/pub', Int8, queue_size=10)
        rospy.Subscriber('mir_api/sub', Int8, self.callback_call_mir)
        self.guid = self.robot.get_mission("GoToGr8")

    def callback_call_mir(self, data):
        if data.data == 1:
            if self.robot.add_mission_to_queue(self.guid):
                self.pub.publish(1)
            else:
                self.pub.publish(0)
        elif data.data == 2:
            if self.robot.read_register(8) == 2:
                self.pub.publish(2)
            else:
                self.pub.publish(0)
        elif data.data == 3:
            self.robot.write_register(8, 1)  # MIR can go
            self.pub.publish(3)
        else:
            self.pub.publish(0)
        return 1

    def is_executing(self):
        if self.robot.is_mission_executing(self.guid):
            self.pub.publish(10)

#---------------------------------------------------------------------------------------




if __name__ == "__main__":
    mir = mir_pubsub()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        mir.is_executing()
        rate.sleep()

    rospy.spin()


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
