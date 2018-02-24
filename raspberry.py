import socket                           # Communication between drone and attached raspberry Pi to the drone (Note: Drone has only USB port. Rasoberry pi can't communicate using wired Connection.)
from picamera import PiCamera           # for raspberry pi Camera 
import os                               # Python can interface with underlying operating system
import time                             # time
import cv2                              # opencv is used for Image Processing. It can read, write and save the images.
from vectors import *                   # It deals vectors
import math                             # math library is used for computing trigonometry
import struct                           # It changes structures of data 
camera = PiCamera()                     # make an object of Picamera
camera.resolution = (1058,600)          # resolution of an Image
camera.start_preview()                  # start the camera
HOST = ''                 # Symbolic name - the local host
PORT = 50007              # Arbitrary non-privileged port
PATH = "/home/pi/Desktop/"      #Raspberry Pi Camera will save the image here and then process it.
YAW_ERROR = 4.7                 #ERROR of yaw (in degrees) 
 
class TargetCounter(object):    #Class for Algorithm
	numTargets = 0
	direction = Vector(0,0,0)
	Folder = ''
	step_size = 0
	def __init__(self, Folder, yaw_error):  
		self.numTargets = 0
		self.Folder = Folder
		self.direction = 0
		self.prev_last_target_x = 0
		self.prev_last_target_y = 0
		self.step_size = 0
		self.yaw_error = yaw_error

	def set_direction(self,direction):      #set the direction
		self.direction = direction

	def set_step_size(self,step_size):      #set the step size
		self.step_size = step_size

        #Brain of the Algorithm
        # Basics of the algorithm is to keep track on the last target of the picture in the next picture, then count the new targets in the new picture.
        # Two images has to overlap so that last target of the picture will be present in the next picture.
        # The distance between two consecutive targets can not be greater than overlap between two pictures.
        # Each image will try to find the last target of the previous image.  
	def count(self,num_Image):              
		if num_Image == 1:                              #First Image will not look for the last target of the previous image.
			self.prev_last_target_x = -1            # Last Target of the previous image (X coordinate)
			self.prev_last_target_y = -1            # Last Target of the previous image (Y coordinate)
		print "Image",num_Image
		image_Path = self.Folder+'Image'+str(num_Image)+'.jpg'          # Imagepath
		image = cv2.imread(image_Path)                                  # read the image
		Current_Contours = self.Find_Contours(image)                    # find the number of targets in the image
		if len(Current_Contours) > 0:                                   # if there is any target in the image then apply the algorithm
			last_Target_x,last_Target_y = self.Find_Last_Target(Current_Contours)   #find the last target of the image
			if self.prev_last_target_x < 0:                                             #If it is first image, then count the total available targets in the image.
				self.numTargets = self.numTargets + len(Current_Contours)               #count the targets
		 	else:
				index, target_center_x, target_center_y = self.Find_Prev_Last_Target(Current_Contours)  #find the last target of the previous image in the new image (Heart of the algorithm)
				if index >= 0:                                                  #if there are new targets
					numNewTargets = self.Count_New_Targets(index, target_center_x,target_center_y) #distinguish new and old targets
					self.numTargets = self.numTargets + numNewTargets       #Count the targets
			self.prev_last_target_x = last_Target_x                         # update the last target of the previous image
			self.prev_last_target_y = last_Target_y                         # update the last target of the previous image
		else:
			self.prev_last_target_x = -1                                    # update the last target of the previous image
			self.prev_last_target_y = -1				        # update the last target of the previous image
		
	def Find_Contours(self,img):
		THRESHOLD_LOW = (0,100,100)                                     # Threshold Low of Red Color
		THRESHOLD_HIGH = (10,255,255)                                   # Threshold High of Red Color
		THRESHOLD_LOW1 = (160,100,100)                                  # Threshold Low of Red Color
		THRESHOLD_HIGH1 = (179,255,255)                                 # Threshold High of Red Color
		MIN_RADIUS = 6
		img_filter = cv2.medianBlur(img,15)                             #Blurr the image
		img_filter = cv2.cvtColor(img_filter, cv2.COLOR_BGR2HSV)        #convert the color to HSV
		img_binary1 = cv2.inRange(img_filter,THRESHOLD_LOW,THRESHOLD_HIGH,0)    #Covert into the binary
		img_binary2 = cv2.inRange(img_filter,THRESHOLD_LOW1, THRESHOLD_HIGH1,0) #Covert into the binary
		img_binary = img_binary1 + img_binary2      
		img_binary = cv2.dilate(img_binary, None, iterations = 6)	        #dilate the image
		contours = cv2.findContours(img_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]         #find contours
		print "number of objects detected", len(contours)               #number of the targets
		return contours
	def Find_Last_Target(self,contours):                    # find the last target of the current image
		distance = []
		x_list = []
		y_list = []
		center = None
		radius = 0
		cmp_x = 1058
		cmp_y = 0
		if len(contours) > 0:	# if at least one object is detected
			for i in range(len(contours)):
				((x, y), radius) = cv2.minEnclosingCircle(contours[i])	#(x,y) is center of object and radius is radius of the object 
				x = int(x)
				y = int(y)
				dist = cmp_x - x  #find the distance from last point of the image from each objects	
				x_list.append(x)			#list all x coordinates in x_list
				y_list.append(y)			#list all y coordinates in y_list
				distance.append(dist)		# list all the distances from object to the end of the image in distance list
				M = cv2.moments(contours[i])		# find momentum of each targets 
				if M["m00"] > 0:		#this method is only for rounding outside the targets
					center = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])) 		
				else:
					print "nothing"
			a = distance.index(min(distance))		# find the last target of the image
			last_point_x = x_list[a]
			last_point_y = y_list[a]
			return last_point_x,last_point_y
	def Find_Prev_Last_Target(self,contours):               #find the last target of the previous image in the current image (Heart of the Algorithm)
		point_new = []
		x_list1 = []
		y_list1 = []
		new_vector_x = []
		new_vector_y = []
		magnitude = []
		center = None
		radius = 0
		if len(contours) > 0:				
			for i in range(len(contours)):
				((x, y), radius) = cv2.minEnclosingCircle(contours[i])
				x = int(x)
				y = int(y)
				x_list1.append(x)
				y_list1.append(y) 
				new_vector_x.append(x-self.prev_last_target_x) 		#find the vector to all the new Targets fron the last object of the previous image (Ghost Target)
				new_vector_y.append(y-self.prev_last_target_y) 		
				new_vector = Vector(new_vector_x[i],new_vector_y[i],0)
				mag = new_vector.magnitude()                            # Find the magnitude of the vector                          
				if mag == 0:
					return -1,0,0
				magnitude.append((abs(self.step_size - mag)/step_size)**2)      # the method to count the error in distance cover by the drone. 
				unit_vector = unit_vector.multiply(1/mag)		#find Unit vector
				point_new.append(unit_vector)		
				M = cv2.moments(contours[i])					 
		deviation = []
		for j in range(len(point_new)):
                        #Direction vector indicates the direction of the drone. It has found using the Yaw value of the drone.
                        #It has found out in Drone. We send that direction from drone to raspberry pi
			d = self.direction.angle(point_new[j])          #The angle between the direction vector and the unit vectors
			deviation.append((abs(d)/self.yaw_error)**2+magnitude[j])       # deviation is weighted sum of the angle and the distance.
		a = deviation.index(min(deviation))				#the point with minimum deviation is last point of previous image	
		x = x_list1[a]
		y = y_list1[a] 
		return a,x_list1,y_list1
	def Count_New_Targets(self,index,target_center_x,target_center_y):      #distinguish between new and old targets
		count1 = 0
		count2 = 0
		new_vector_x = []
		new_vector_y = []
		point_new1 = []
		new = 0
		for i in range(len(target_center_x)):					
			unit_vector = Vector(target_center_x[i] - target_center_x[index],target_center_y[i] - target_center_y[index],0)
			mag = unit_vector.magnitude()
			if mag != 0:		 
				unit_vector = unit_vector.multiply(1/mag)						
			a = self.direction.dot(unit_vector)	# if the dot product of the direction and unit vectors is negative , then it is the New Target
                                                                # if the dot product of the direction and unit vectors is positive , then it is the Old Target
			if a < 0:
				new = new + 1
				print "new object detected in function",new
			else:
				pass
		return new
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))	
s.listen(1)
conn, addr = s.accept()
print 'Connected by', addr
frame = 0
Artesia_Counter = TargetCounter(PATH,YAW_ERROR)                 # create the object for the algorithm
while 1:
	frame = frame + 1                               
	data = conn.recv(1024)                          #receiving the data from the drone (Data : Direction of the drone, Step size of the drone)
	if data == 'stop':
		conn.close()
		break
	direction, step_size = struct.unpack('ff',data)         # unpack the data received from the drone
	print "direction "+str(frame)+"= ", direction   
	direction = math.radians(direction)     
	x = math.cos(direction)
	y = math.sin(direction)
	direction = Vector(x,y,0)                       #create the direction vector
	print "direction vector"+str(frame)+"=", direction
	Artesia_Counter.set_direction(direction)        # set the direction
	Artesia_Counter.set_step_size(step_size)        # set the step size
	camera.capture(PATH+"Image"+str(frame)+".jpg")
	Artesia_Counter.count(frame)            #apply the algorithm
	conn.send('pic_taken')
	
print "Total number of targets",Artesia_Counter.numTargets #Toatal number of the targets
camera.stop_preview()           #stop the camera preview
conn.close()


