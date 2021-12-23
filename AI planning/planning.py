# ***************************
#  *  planning.py
#  * Version Number  : 1.0
#  * Configuration Identifier: 
#  * Modified by:  Shrilesh Kale       
#  * Modified Date:  22/12/2021       
#  * Description: Source code for generating AI planner's plan
#  * Note- This code was not tested so it might contain some errors.
#  **************************/

import json
import requests
import sys
import os
import paho.mqtt.client as mqtt
import time

# Data to pass to online editor of pddl
data = {
        'domain': open(sys.argv[1], 'r').read(),
        'problem': open(sys.argv[2], 'r').read()

    }


# Function for MQTT connect ,triggrs when connection established
def on_connect(mqttc, obj, flags, rc):
    print("rc: " + str(rc))
    print('Connected')


# Function on_message is callback function once data is received
def on_message(mqttc, obj, msg):
    response = msg.payload["topic"]

    if response == "ble_sense/sensor/gesture":
        gesture_data = (str(response.decode("utf-8")))
       
    elif response == "ble_sense/sensor/microphone":
        microphone_data = (str(response.decode("utf-8")))
       
    elif response == "ble_sense/sensor/proximity":
        proximity_data = (str(response.decode("utf-8")))

    if gesture_data != "":
        if microphone_data != "":
            if proximity_data != "":
                sensors = {

                    "gesture": gesture_data,
                    "microphone": microphone_data,
                    "proximity_data": proximity_data

                    }
                 # Data to pass to online editor of pddl
                    data = {
                    'domain': open(sys.argv[1], 'r').read(),
                    'problem': open(sys.argv[2], 'r').read(),
                    'sensors': sensors

                    }
    else:
        print("Data is empty")

    get_plan(data)
    fileop()

   

# Function which triggers when subscribed to particular topic
def on_subscribe(mqttc, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))
    return

# Function to publish data
def publish_data():

    Data = {
        'actuation': 'ON'
    }
    mqttc.publish('ai_planner/actuation/receive', json.dumps(Data))
    time.sleep(1)


# Function to execute AI planning and get plan.txt file
def get_plan(data):
    response = requests.post('http://solver.planning.domains/solve', json=data).json()
    with open(sys.argv[3], 'w') as f:
        for act in response['result']['plan']:
            f.write('\n')
            f.write(str(act['name']))

# Function to operate on plan.txt file
def fileop():
    f = open("plan.txt", "r")
    zero_line = f.readline()
    first_line = f.readline()
    second_line = f.readline()
    third_line = f.readline()
    if first_line[7] == 'o' and first_line[8] == 'n':
        if second_line[5] == 'o' and second_line[6] == 'n':
            if third_line[5] == 'o' and third_line[6] == 'n':
                publish_data()

    f.close()


# Initialize MQTT and subscribe to MQTT broke(RPi)
mqttc = mqtt.Client("client-id")
mqttc = mqtt.Client()
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.connect("broker_ip", 1883, 60)
mqttc.subscribe("ble_sense/sensor/gesture", 0)
mqttc.subscribe("ble_sense/sensor/microphone", 0)
mqttc.subscribe("ble_sense/sensor/proximity", 0)

mqttc.loop_forever()
