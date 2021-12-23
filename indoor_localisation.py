# /***************************
#  *  controller.py
#  * Version Number	: 1.0
#  * Configuration Identifier: 
#  * Developed by:  Shrilesh Kale       
#  * Modified Date:  22/12/2021       
#  * Description: Source code for backend operations for Indoor positioning 
#  **************************/

# -*- coding: utf-8 -*-
import time
import os
import sys
from beacontools import BeaconScanner, IBeaconFilter
import math
from heapq import nsmallest
import functools
import json
from bson import json_util
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import telepot
import grovepi
import itertools
import multiprocessing
from multiprocessing import Pool,Process
import threading
GPIO.setmode(GPIO.BOARD)
temp_sensor = 2
blue = 7
light_sensor = 0
threshold = 10
grovepi.pinMode(light_sensor,"INPUT")
normalLightLed = 5
grovepi.pinMode(normalLightLed,"OUTPUT")

 #Output on the pin for 
# GPIO.setup(light_sensor, GPIO.IN)

a1 = "b9407f30-f5f8-466e-aff9-25556b57feff" # blueberry
a2 = "b9407f30-f5f8-466e-aff9-25556b57fe6f"#Mint
a3 = "b9407f30-f5f8-466e-aff9-25556b57fe77" # ice
a4 = "b9407f30-f5f8-466e-aff9-25556b57fe6d" #coconut

#-72,-69,-61,-60 dummy values
beacon1Rssi = []
beacon2Rssi = []
beacon3Rssi = []
beacon4Rssi = []
filteredBeacon1RssiValues =[]
filteredBeacon2RssiValues =[]
filteredBeacon3RssiValues =[]
filteredBeacon4RssiValues =[]
x_data = 0
y_data = 0
bt_addr = 0
rssi = 0 
packet = 0 
additional_info = 0
listtofuniquevalues = []
listt = []
beacon_coordinates = {
	'1' : [1,1],
	'2' : [1,10],
	'3' : [10,10],
	'4': [10,1]
}

# Check for  Command line argument 
try: 
	if sys.argv[1] != ( 0  or ''):
		print 'Ip address given is:-'+str(sys.argv[1])
		ipAddr = sys.argv[1]

except Exception, e:
	print 'Please provide IP address of Broker as a parameter like e.g. python server.py 192.168.0.101'
	print ("Error:{}".format(e))
	raise e

# *******************************************************************************
  # Function    : on_connect
  # Description : Function to let user know when MQTT connection is established
  #               
  # Input       : mqtt object ,rc flag 
  # Output      : NA
  # Return      : NA
# *******************************************************************************/
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

# *******************************************************************************
  # Function    : on_message
  # Description : Function for MQTT connect ,triggrs when connection established
  #               
  # Input       : mqtt object ,rc flag 
  # Output      : NA
  # Return      : NA
# *******************************************************************************/

def on_message(client, userdata, msg):
    print(msg.topic+" "+ (str(msg.payload)).decode("utf-8") )
    

client = mqtt.Client()
client.connect(ipAddr, 1883)
client.on_connect = on_connect
client.on_message = on_message
client.loop_start()

# Inspired from  
# *******************************************************************************
  # Class    : Kalman Filter
  # Description : Class for Kalman filter for filtering RSSI values
  #               
  # Input       : R and Q 
  # Output      : NA
  # Return      : NA
# *******************************************************************************/

#The process noise R which describes noise caused by the system itself.
#For the RSSI example we use a low value for the process noise (e.g. 0.008);
#we assume that most of the noise is caused by the measurements.

#R models the process noise and describes how noisy our system internally is. Or, in other words,
# how much noise can we expect from the system itself? As our system is dynamic we can set this to a high value.

#Q resembles the measurement noise; how much noise is caused by our measurements? 
#As we expect that our measurements will contain most of the noise,
#it makes sense to set this parameter to a high number

class KalmanFilter:
    cov = float('nan')
    x = float('nan')
    def __init__(self, R, Q):
        """
        Constructor
        :param R: Process Noise
        :param Q: Measurement Noise
        """
        self.A = 1
        self.B = 0
        self.C = 1
        self.R = R
        self.Q = Q
    def filter(self, measurement):
        """
        Filters a measurement
        :param measurement: The measurement value to be filtered
        :return: The filtered value
        """
        u = 0
        if math.isnan(self.x):
            self.x = (1 / self.C) * measurement
            self.cov = (1 / self.C) * self.Q * (1 / self.C)
        else:
            predX = (self.A * self.x) + (self.B * u)
            predCov = ((self.A * self.cov) * self.A) + self.R
            # Kalman Gain
            K = predCov * self.C * (1 / ((self.C * predCov * self.C) + self.Q));
            # Correction
            self.x = predX + K * (measurement - (self.C * predX));
            self.cov = predCov - (K * self.C * predCov);
        return self.x
    def last_measurement(self):
        """
        Returns the last measurement fed into the filter
        :return: The last measurement fed into the filter
        """
        return self.x
    def set_measurement_noise(self, noise):
        """
        Sets measurement noise
        :param noise: The new measurement noise
        """
        self.Q = noise
    def set_process_noise(self, noise):
        """
        Sets process noise
        :param noise: The new process noise
        """
        self.R = noise


# *******************************************************************************
  # Function    : callback 
  # Description : Function for scanning RSSI values and applying kalman filter to it
  #               
  # Input       : Bluetooth packet
  # Output      : Filtered RSSI values
  # Return      : NA
# *******************************************************************************/


def callback(bt_addr, rssi, packet, additional_info):

	global beacon1Rssi,beacon2Rssi,beacon3Rssi,beacon4Rssi
	global filteredBeacon1RssiValues,filteredBeacon2RssiValues,filteredBeacon3RssiValues,filteredBeacon4RssiValues
	deleteArray = [beacon1Rssi,beacon2Rssi,beacon3Rssi,beacon4Rssi]
	deleteFilteredArray = [filteredBeacon1RssiValues,filteredBeacon2RssiValues,filteredBeacon3RssiValues,filteredBeacon4RssiValues]
	kalObj = KalmanFilter(0.01,0.001) # Object creation for Kalman filter class
	# 0.01,0.001
	try:
		if bt_addr == "c1:63:db:21:15:e3": # Beacon-1 (Mint)
			# xx = [rssi for x in range (5) if len(beacon1Rssi) < 5]
			if len(beacon1Rssi) < 5:
				beacon1Rssi.append(rssi)

				
			
		if bt_addr == "c5:5d:e4:cd:19:ea": # Beacon-3 (ice)
			if len(beacon2Rssi) < 5:
				beacon2Rssi.append(rssi)
				
			

		if bt_addr == "ea:d2:a2:af:56:bd": # Beacon-2 (Blueberry)
			if len(beacon3Rssi) < 5:
				beacon3Rssi.append(rssi)
				
			
			
		if bt_addr == "e2:bf:b4:e7:97:9f":  # Beacon-4 (coconut)
			if len(beacon4Rssi) < 5:
				beacon4Rssi.append(rssi)
				
			
		if (len(beacon1Rssi) and len(beacon2Rssi) and len(beacon3Rssi) and len(beacon4Rssi)) == 5:

			for (a, b, c, d) in itertools.izip_longest(beacon1Rssi, beacon2Rssi, beacon3Rssi,beacon4Rssi): 
				if (a and b and c and d) is not None:
					x = kalObj.filter(a)
					rounded = round(x, 2)
					filteredBeacon1RssiValues.append(kalObj.filter(a))
					filteredBeacon2RssiValues.append(kalObj.filter(b))
					filteredBeacon3RssiValues.append(kalObj.filter(c))
					filteredBeacon4RssiValues.append(kalObj.filter(d))

			XandYCoordinateCal(filteredBeacon1RssiValues,filteredBeacon2RssiValues,filteredBeacon3RssiValues,filteredBeacon4RssiValues)
			# XandYCoordinateCal(beacon1Rssi,beacon2Rssi,beacon3Rssi,beacon4Rssi)
			for i,j in itertools.izip_longest(deleteArray, deleteFilteredArray):
				del i[:]
				del j[:]

	except Exception as e:
		print 'Failed to get the RSSI values'
		raise e


# *******************************************************************************
  # Function    : scanOptimized
  # Description : Function to start scanning RSSI values of BLE Beacons
  #               
  # Input       : NA
  # Output      : NA
  # Return      : NA
# *******************************************************************************/

def scanOptimized():
	scanlist =[a1,a2,a3,a4]
	cnt = 0
	for i in scanlist:
		cnt = cnt +1
		RSSI_Scan =  BeaconScanner(callback,device_filter=IBeaconFilter(uuid=i)) 
		RSSI_Scan.start()

# *******************************************************************************
  # Function    : average
  # Description : Function to calculate average
  #               
  # Input       : list
  # Output      : NA
  # Return      : average of list
# *******************************************************************************/

def average(l): 
    avg = reduce(lambda x, y: x + y, l) / len(l)
    return avg

# *******************************************************************************
  # Function    : XandYCoordinateCal
  # Description : Function to calculate x and y values for location estimation
  #               
  # Input       : Filtered RSSI values of 4 beacons
  # Output      : NA
  # Return      : X and Y coordinate
# *******************************************************************************/

def XandYCoordinateCal(filter1,filter2,filter3,filter4):
	listForEuclideanDist =[]
	finalCalculation = []
	global x_data,y_data, beacon_coordinates
	beacon_id = [1,2,3,4]
	try:

		if ( len(filter1) and len(filter2) and len(filter3) and len(filter4) ) != 0:

			AverageValueofBeacon1 = average(set(filter1))
			AverageValueofBeacon2 = average(set(filter2))
			AverageValueofBeacon3 = average(set(filter3))
			AverageValueofBeacon4 = average(set(filter4))
			
			arr = []
			E1 = math.sqrt( (AverageValueofBeacon1 +40)**2  + (AverageValueofBeacon2 +72)**2+ (AverageValueofBeacon3 +62)**2  + (AverageValueofBeacon4 +81)**2 )
			arr.append(E1)
			E2 = math.sqrt( (AverageValueofBeacon1 +70)**2  + (AverageValueofBeacon2 +40)**2+ (AverageValueofBeacon3 +59)**2  + (AverageValueofBeacon4 +85)**2 )
			arr.append(E2)
			E3 = math.sqrt( (AverageValueofBeacon1 +60)**2  + (AverageValueofBeacon2 +65)**2+ (AverageValueofBeacon3 +40)**2  + (AverageValueofBeacon4 +63)**2 )
			arr.append(E3)
			E4 = math.sqrt( (AverageValueofBeacon1 +59)**2  + (AverageValueofBeacon2 +77)**2+ (AverageValueofBeacon3 +72)**2  + (AverageValueofBeacon4 +40)**2 )
			arr.append(E4)

			arr, beacon_id = zip(*sorted(zip(arr, beacon_id)))

			smallestOfEuclideanDist = arr[0:3] # to get the minimum 3 values from array of size 4
			smallestOfEuclideanDist_id = beacon_id[0:3]
			# print smallestOfEuclideanDist_id
			denominator = (1/(smallestOfEuclideanDist[0]**2)) + (1/(smallestOfEuclideanDist[1]**2)) + (1/(smallestOfEuclideanDist[2]**2))		
			calculation = [0,0,0]
			x = 0
			y = 0
			for idx, value in enumerate(smallestOfEuclideanDist):
				calculation[idx] = (1/(value)**2) / denominator
				
			for idx,every_beacon in enumerate(smallestOfEuclideanDist_id):
				x += beacon_coordinates[str(every_beacon)][0] * calculation[idx]
				y += beacon_coordinates[str(every_beacon)][1] * calculation[idx]

			x_data = round(x,2)
			y_data = round(y,2)
			publishLocationData(x_data,y_data)

			if ( (7 <= x_data <= 10) and (y_data >= 8) ):

				publishLocation('Location near beacon-3')
			
			if ( (x_data <= 5) and (y_data <= 5)):

				publishLocation('Location near exit area')

			if ( (x_data >= 8) and (y_data <= 4)):

				publishLocation('Location near beacon-4')#beacon-4

			elif ( (5 <= x_data <= 8 ) and (5 <= y_data <= 8) ):
				publishLocation('no where around preferred area')

			return x_data, y_data

		else:

			print("Filtered values not found")

	except Exception as e:
		print 'Failed to calculate X and Y coordinates'
		raise e

# *******************************************************************************
  # Function    : publishLocationData
  # Description : Function to publish x and y coordinate
  #               
  # Input       : x and y coordinate
  # Output      : NA
  # Return      : NA
# *******************************************************************************/

def publishLocationData(x_data, y_data):
	
	locationData = {
		
		'x_coordinate': x_data, 
		'y_coordinate': y_data
		}

	print(locationData)
	client.publish('location_data/xandy/device_1',json.dumps(locationData))
	time.sleep(1)

# *******************************************************************************
  # Function    : PublishLocation
  # Description : Function to publish approximate user location (e.g whether user is near to beacon or not?)
  #               
  # Input       : approximate user location
  # Output      : NA
  # Return      : NA
# *******************************************************************************/

def publishLocation(locationData):

	location =  {

		'userlocation': locationData
	}
	print(location)
	client.publish('location_data/userlocation/device_1',json.dumps(location))

# Start scanning RSSI values
scanOptimized()




