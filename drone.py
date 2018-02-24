#include all the reqired libraries

import socket         # Communication between drone and attached raspberry Pi to the drone (Note: Drone has only USB port. Rasoberry pi can't communicate using wired Connection.)
import time         #   Time 
import math         # math library is used for computing trigonometry
from dronekit import connect, VehicleMode, LocationGlobalRelative   # dronekit is used for autonomous flights of the drone
from pymavlink import mavwp     # library to control the flight Controller
import numpy as np      # numpy 
import struct       # It changes structures of data

HOST = '10.1.1.130'    # The remote host - Raspberry PI
PORT = 50007              # The same port as used by the server
DISTANCE = 15           #Meters --> Total flight length 
STEP_SIZE = 3           #Meters --> The drone covers step size distance to capture next image
OVERLAP = 1             # Meters --> Overlap between two images
GPS_ERROR = 1           # Meters --> GPS Error has to considered
CONST_HORI = 1.1438     # This constant is derived from Horizontal field of View of the camera.
                        # Horizontal Field Of View = HOV
                        # Total_Image_Size = Step_Size + Overlap + GPS_ERROR
                        # To find the height of the drone, we consider the Horizontal Field Of View.
                        # CONST_HORI = (Total_Image_Size/2)*(2 * tan(HOV/2))

ALT = (STEP_SIZE+OVERLAP+GPS_ERROR)/CONST_HORI          # Calculation of Altitude
print "Altitude",ALT 
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)   # initialise socket
s.connect((HOST, PORT))                 # connect socket with HOST

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
connection_string = '127.0.0.1:14550'

print 'Connecting to vehicle on: %s', connection_string
vehicle = connect(connection_string, wait_ready=True)  # Connect vehicle with Drone Controller


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
	print " Waiting for vehicle to initialise..."
	time.sleep(1)


    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True    

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:      
	print " Waiting for arming..."
	time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
	print " Altitude: ", vehicle.location.global_relative_frame.alt 
	#Break and return from function just below target altitude.        
	if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
	#time.sleep(5) 
    		print "Reached target altitude"
    		break
	time.sleep(1)
arm_and_takeoff(ALT)
dist = 0

while 1:
	a = vehicle.location.global_frame.lon       #get Longitude and Latitude
	b = vehicle.location.global_frame.lat
	print "longitude=",a
	print "latitude=",b
	lat = 0
	lon = 0.000011503*STEP_SIZE # move in this direction with the distance of STEP_SIZE. (in degrees)
	new_lat = b + lat
	new_lon = a + lon
	print "Going towards east point"
	point1 = LocationGlobalRelative(new_lat, new_lon, ALT) #set destination accordinag to the step size of the drone.
	vehicle.simple_goto(point1)     #give command to the drone so it will go in that direction
	time.sleep(5)                   #give it 5 seconds to reach there
	yaw = vehicle.attitude.yaw      #get yaw 
	yaw = math.degrees(yaw)         #convert it in degrees
	alpha = math.degrees(np.arctan2(lat,lon))      # It is the direction of the drone in Polar Coordinate System
	if alpha < 0:                   
		alpha = alpha + 360                     
	alpha = alpha + 90                              # The camera attached with the drone is 90 degrees rotated respect to the drone
	direction = yaw + alpha                         # actual direction of the picture captured by the raspberry pi
	dist = dist + STEP_SIZE                          
	data = struct.pack('ff',direction,STEP_SIZE)    #pack direction and step size in one packet
	s.send(data)                        #send that packet to the raspberry pi
	data = s.recv(1024)                 # wait for the acknowledgement from the raspberry pi
	if data == 'pic_taken':                 
		print "confirmation : pic has taken"    
		if dist >= DISTANCE:        #The loop will be repeated untill the drone covers the whole path.
			print "Returning to Land"               
			vehicle.mode = VehicleMode("LAND")      # landing command to the drone
			#Close vehicle object before exiting script
			print "Close vehicle object"
			vehicle.close()
			s.send('stop')
			break
		else:
			pass
	else:
		print "did not get the echo"	

s.close()


