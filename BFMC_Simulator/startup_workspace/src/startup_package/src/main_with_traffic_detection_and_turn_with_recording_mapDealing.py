#!/usr/bin/python

"""
These are the libraries you need to import in your project in order to
be able to communicate with the Gazebo simulator
"""
# Only for use in Python 2.6.0a2 and later
from __future__ import print_function
from bfmclib.gps_s import Gps
from bfmclib.bno055_s import BNO055
from bfmclib.camera_s import CameraHandler
from bfmclib.controller_p import Controller
from bfmclib.trafficlight_s import TLColor, TLLabel, TrafficLight
from bfmclib.controller_p import get_current_steer, get_current_speed
import bfmclib.controller_p
import numpy as np
import bs4 as bs
import threading
import rospy
import cv2
from time import sleep
import time
from keras.models import load_model
from pynput.keyboard import Key, Listener
from datetime import datetime
from math import sqrt
from PIL import Image
import os
import csv



# This line should be the first line in your program
rospy.init_node('main_node', anonymous=True)

# build object from the car handler
cam = CameraHandler()
print("Camera loaded")
# build object from the car controller
car = Controller()
print("Controller loaded")

# create a GPS object
gps = Gps()
print(" Gps created ")

# create TrafficLight object
traffic_light = TrafficLight()
print(" traffic lights  created ")

# creating a BNO object
bno = BNO055()
print("Bno created")

# location of the 4 traffic signs based on the given map
# TL0 = [1 , 14.5] # TL1 = [2 , 11.2] # TL2 = [3.7,10.3] # TL3 = [3.3,11.5]
traffic_light_locations = { 'TL0': [1 , 14.5], 'TL2':[2 , 11.2],
							'TL1' :[3.7,10.3],'TL3':[3.3,11.5]    }

# some global varible to deal with person reaction
last_excution_person_stop = True
last_excution_person_stop_time = 0

# some global varible to deal with cross walk reaction
last_excution_cross_walk = True
last_excution_cross_walk_time = 0

# some global varible to deal with tail close car reaction
last_excution_tail = True
last_excution_tail_time = 0

# define a variable as a lock
lock = "free"


# if you want to exit frame preview
print("Select \"Frame preview\" window and press Q to exit")
print("Select \"Frame preview\" window and press P to stop the recording")
print("Select \"Frame preview\" window and press R to resume the recording")






"""								 open and process the digital map 					"""
# open the map in xml format
map_path = os.getcwd() + "/map/map.xml"
xml_file = open(map_path,'r')
# building a soup object to parse
map = bs.BeautifulSoup( xml_file , 'xml')
# extracting all nodes tags and build a dictinary contain all nodes id with
# its related coordination
nodes_dict = {}
for node in map.find_all('node'):
	# get all data tags from the node tag
	x,y = node.find_all('data')
	# get the value of each data tag
	x  =  eval(str(x.text))
	y  =  eval(str(y.text))
	# get the id of the each node
	id =  eval(str(node['id']))
	# add element to the dictinary contain  { key = id : value = [x,y] }
	nodes_dict[ id ]   =  [ x , y ]
# extracting all edges tags and build a dictinary contain all edges and
# the situation of ech edge
edges_dict = {}
for edge in map.find_all('edge'):
	# get  the value of data tag from the edge tag
	status = edge.data.text
	status = str( status )
	# get the value of source and target attrebutes in the edge tag
	source =  eval(str(edge['source']))
	target =  eval(str(edge['target']))
	# add element to the dictinary contain  { key = (source,target) : value = status }
	edges_dict[ (source,target) ]   =  status

# Enter the distination coordinations
dis = raw_input("please Enter the distination coordinates: ex 3.5 1.2 \n ")
x_y =  dis.split()
x_dis = eval( x_y[0] )
y_dis = eval( x_y[1] )








"""                       Define the main preprocess function               """
# to preprocess the image before making a prediction
def img_preprocess(img, driving_mode):
	# go ahead
	if driving_mode == 3 :
		img = img[200:,:,:]
	# if the mode is turning left and right and change lane
	elif driving_mode in[1,2,4]:
		img = img[120:, :, :]
	elif driving_mode == 5:
		img = img[150:, 250:, :]

	elif driving_mode == 7:
		img = img[200:, 250:, :]
	else :
		img = img[150:,:,:]

	img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
	img = cv2.GaussianBlur(img,  (3, 3), 0)
	img = cv2.resize(img, (200, 66))
	# show the image on the previw
	cv2.imshow("process ", img)
	key = cv2.waitKey(1)
	if key == ord('c'):
		cv2.destroyAllWindows()
	img = img/255
	img = img.reshape(66, 200, 1)
	return img



# stop sign hanlding method thread
def stop_reaction(last_excution_time):
	# get current time in seconds to compared with the last excution
	current_time = round(time.time(), 2)
	# if there is a more than 5 seconds between the last execution and the this one
	if (current_time - last_excution_time) > 5:
		# save the previos speed
		previous_speed = get_current_speed()
		# make the care stop for 3 seconds
		car.drive( 0.0 , get_current_steer() )
		sleep(3)
		# return the previous_speed to the car
		car.drive( previous_speed , get_current_steer() )



# traffic light reaction thread
def traffic_light_reaction(car_postion):
	# 0 means red, 1 means yellow, 2 means green
	# some testing to know what is the closest traffic point to car postion
	# define a start value to be the minmal distance
	min_distance = 1000
	traffic_id = 'TL0'
	# iterate over all items of the traffic lights
	for (key,value) in traffic_light_locations.items():
		# some basic calculations to figure out the distance
		# between the current car location and the current point
		v1 = (value[0] - car_postion[0]) ** 2
		v2 = (value[1] - car_postion[1]) ** 2
		current_distance = sqrt(v1 + v2)
		# save the key of the point that has the minmal distance value
		if current_distance < min_distance:
			traffic_id = key
			min_distance = current_distance
	# get the status of the closest traffic sign
	traffic_status = None
	closest_traffic_sign = traffic_id
	if closest_traffic_sign == "TL0":
		traffic_status = traffic_light.getTL0State()
	elif closest_traffic_sign == "TL1":
		traffic_status = traffic_light.getTL1State()
	elif closest_traffic_sign == "TL2":
		traffic_status = traffic_light.getTL2State()
	elif closest_traffic_sign == "TL3":
		traffic_status = traffic_light.getTL3State()
	print( traffic_id,"status: ", traffic_status)
	# wait until the green color (traffic_status=0)
	if  traffic_status == 0:
		# move the car
		car.drive( 0.4 , get_current_steer() )
	else :
		# stop the car
		car.drive( 0 , get_current_steer() )




# when there is a pedestrian in the road
def pedestrian_reaction():
	# take the lock
	global lock
	lock = "taken"
	# stop the car
	car.drive(0, get_current_steer())
	# make these variable global to effect the main variables
	global last_excution_person_stop
	global last_excution_person_stop_time
	# change last_excution_person_stop to false and save the last excution time
	last_excution_person_stop = False
	last_excution_person_stop_time = round(time.time(), 2)


# a thread function to react on cross walk
def cross_walk_reaction():
	# slow dawn the car
	car.drive(0.2, get_current_steer())
	# make these variable global to effect the main variables
	global last_excution_cross_walk
	global last_excution_cross_walk_time
	# change last_excution_cross_walk to false and save the last excution time
	last_excution_cross_walk = False
	last_excution_cross_walk_time = round(time.time(), 2)


# a thread to react on close car
def tail_stop_reaction():
	# stop the car
	car.drive(0.0, get_current_steer())
	# make these variable global to effect the main variables
	global last_excution_tail
	global last_excution_tail_time
	# change last_excution_cross_walk to false and save the last excution time
	last_excution_tail = False
	last_excution_tail_time = round(time.time(), 2)



# function to be multithreading with the control thread
def images_thread():
	"""  								 loading all the necessary AI models    						 """

	# load the lane keeping model
	model = load_model('model_lane_keeping.h5')
	# load the  turn right model
	model_turn_right = load_model('model_turn_right_aug_120_v4.h5')
	# load the  turn left model
	model_turn_left = load_model('model_turn_left_aug_120_v2.h5')
	# load the  turn go ahead model
	model_turn_go_ahead = load_model('model_go_ahead.h5')
	# load the change lane left
	model_change_lane_left = load_model('model_change_lane_left_aug_120_v6.h5')
	# to keep lane in highway left
	model_keep_lane_left = load_model('model_keep_for_left_aug_120_v15.h5')
	# load the take over
	model_change_lane_right = load_model('model_keep_for_change_lane_right_aug_150_v5.h5')
	# load park vertical model
	model_parknig_vertical = load_model('model_keep_for_parking_vertical_v2.h5')



	"""  								Some important variables    						 """

	# defining list to hold all frames and related steering angles
	image_paths = []
	steering_list = []
	# a variable to hold the recording status
	record = False
	# to name the images
	im_number = 0
	# a variable to save driving  mode status
	# 0 means lane keeping, 1 means turn right, 2 means turn left
	# 3 means go a head, 4 means take over
	driving_mode = 0
	# counter to limit  frame per second when using the model
	# to not over helm the processor
	limit_frames = 0
	# a variable to allow car ver passing on high way
	allow_car_passing = False
	# counter to pass until the turn is made
	turn_counter = 0
	# to use when making a turn decision manually
	manual_turn_command = ''
	# make the driving mode is manual at start by default
	manual_driving = True
	# make the defalt value of parking mode is off
	parking_mode = False
	# default timer value for stop sign reaction
	last_excution_time = 0
	# car postion default value
	car_postion = [0,0]
	# make these varible visiable in all these thread
	global last_excution_person_stop
	global last_excution_person_stop_time
	global last_excution_cross_walk
	global last_excution_cross_walk_time
	global last_excution_tail_time
	global last_excution_tail
	global lock





	"""  								Loading Yolo version 3 object detection Model  						 """


	# Load Yolo trained model for traffic detection and its configuration
	# we are using opencv library version 4
	net = cv2.dnn.readNet("yolov3-tiny_final.weights", "yolov3-tiny.cfg")

	# load the classes for the traffic detection from classes.names file
	with open("classes.names", "r") as f:
		classes = [line.strip() for line in f.readlines()]

	# some config for the detection model
	# we want to gwt the output of the last layers
	layer_names = net.getLayerNames()
	output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]





	"""								 Main While loop 							"""
	while True:

		# by default it is False
		if record :
			# to record 10 fps
			if limit_frames :
				# get the exact date to combign it with image name
				exact_time = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p")
				# build the directory and the name of each image
				im_name = os.getcwd() + "/DataSet/IMG/img_{}{}.jpg".format(exact_time,im_number)
				# # using Image class fron PIL modul to get and save the image
				# im = Image.fromarray( cam.getImage() )
				# im.save(im_name)
				cv2.imwrite(im_name, cam.getImage() )
				# append image by imag path
				image_paths.append(im_name)
				# increasing the image counter number, to makesure all the images
				# have diffrent names
				im_number += 1
				# images are (480, 640, 3)
				# print(cam.getImage().shape)
				# get the steering angle on every frame
				steering_list.append( bfmclib.controller_p.get_current_steer())


		# Loading image
		img =  cam.getImage()
		# preprocess the current frame
		image = img_preprocess(img, driving_mode)
		image = np.array([image])


		# between 0 to 3 and -3 to 0
		yaw = bno.getYaw()
		# test the GPS methods
		gps_data_dic = gps.getGpsData()
		# time_in_seconds = gps_data_dic['timestamp']
		coordination = gps_data_dic['coor']
		# process data to get car postion
		if coordination:
			# get postion on x and y axis
			x = coordination[0].real
			y = coordination[0].imag
			car_postion = [x,y]


		"""   Choosing what AI model to use based on the Driving mode      """
		# means make a turn right
		if driving_mode == 1:
			# using the turn right model to predict
			# when the counter reaches 100 the car will finish the turn right mode
			if turn_counter > 100:
				# return to lane keeping mode
				# return the manual turn command to null
				manual_turn_command = ''
				driving_mode = 0
			# predict the steering angles
			pred_steering = float(model_turn_right.predict(image))
			print("Turn right is active")
			turn_counter += 1

		# means a turn left
		elif driving_mode == 2:
			# using the turn left model to predict
			# when the counter reaches 150 the car will finish the turn left mode
			if turn_counter > 150:
				# return to lane keeping mode
				# return the manual turn command to null
				manual_turn_command = ''
				driving_mode = 0
			# predict the steering angles
			pred_steering = float(model_turn_left.predict(image))
			print("Turn left is active")
			turn_counter += 1


		# means make go ahead turn
		elif driving_mode == 3:
			# using the go ahead model to predict
			# when the counter reaches 100 the car will finish the go ahead mode
			if turn_counter > 100:
				# return to lane keeping mode
				# return the manual turn command to null
				manual_turn_command = ''
				driving_mode = 0
			# predict the steering angles
			# reused the lane keeping for going ahead on intersection
			pred_steering = float(model_turn_go_ahead.predict(image))
			print("Go ahead is active")
			turn_counter += 1


		# means make change lane left
		elif driving_mode == 4:
			# using the go ahead model to predict
			# when the counter reaches 100 the car will finish the go ahead mode

			if turn_counter > 30:
				# return to left lane keeping mode
				# return the manual turn command to null
				manual_turn_command = ''
				driving_mode = 5
				# reset turn counter
				turn_counter = 0
			# predict the steering angles
			# reused the lane keeping for going ahead on intersection
			pred_steering = float(model_change_lane_left.predict(image))
			print("change lane left is active")
			turn_counter += 1


		elif driving_mode == 5:
			# using the go ahead model to predict
			# when the counter reaches 100 the car will finish the go ahead mode

			if turn_counter > 100:
				# return to left lane keeping mode
				# return the manual turn command to null
				manual_turn_command = ''
				driving_mode = 6
				# reset turn counter
				turn_counter = 0
			# predict the steering angles
			# reused the lane keeping for going ahead on intersection
			pred_steering = float(model_keep_lane_left.predict(image))
			print("keep left lane is active")
			turn_counter += 1


		elif driving_mode == 6:
			# using the go ahead model to predict
			# when the counter reaches 100 the car will finish the go ahead mode

			if turn_counter > 40:
				# return to left lane keeping mode
				# return the manual turn command to null
				manual_turn_command = ''
				driving_mode = 0
			# predict the steering angles
			# reused the lane keeping for going ahead on intersection
			pred_steering = float(model_change_lane_right.predict(image))
			print("change lane right")
			turn_counter += 1

		# means park vertical
		elif driving_mode == 7:
			# using the park vertical model to predict
			# when the counter reaches 100 the car will finish the go ahead mode
			if turn_counter > 500:
				# return to lane keeping mode
				# return the manual turn command to null
				manual_turn_command = ''
				driving_mode = 0
			# predict the steering angles
			pred_steering = float(model_parknig_vertical.predict(image))
			print("park vertical is active")
			turn_counter += 1

		# keep the lane
		else:
			# using the lane keeping model to predict
			# reset the turn counter
			turn_counter = 0
			pred_steering = float(model.predict(image))


		"""                         Object detection and reaction and predict steering           """
		# detection part
		# to reduce the overhead on the cpu
		# we will only apply the detection on 2 frames per seconds
		if limit_frames :

			"""					Predict steering          """
			# sending commands based on the predicting models
			# when the manual driving is off
			if not manual_driving:

				# if a recover from a close person stop
				# check last_excution_person_stop and time diffrence betwwen now and las excution time
				# if it was more than 2 seconds, increase the speed and return last_excution_person_stop to True
				if (not last_excution_person_stop ) and ( round(time.time(), 2) - last_excution_person_stop_time  > 2 ):
					car.drive( 0.4, pred_steering )
					last_excution_person_stop = True
					lock = 'free'

				# if recovering from cross walk
				elif (not last_excution_cross_walk ) and ( round(time.time(), 2) - last_excution_cross_walk_time  > 6 ):
					# make sure that the lock is free
					if lock == 'free':
						car.drive( 0.4, pred_steering )
						last_excution_cross_walk = True

				# if recovering from tail
				elif (not last_excution_tail ) and ( round(time.time(), 2) - last_excution_tail_time  > 1 ):
					car.drive( 0.4, pred_steering )
					last_excution_tail = True

				# depending on the driving mode give predicted steering amgles with the current speed
				car.drive( get_current_speed() , pred_steering )
			print(get_current_speed() , get_current_steer() )


			"""							Draw rectangles on detecting objects    			      """
			# graping the properties of the image
			height, width, channels = img.shape
			# Detecting objects
			#  first we need to convert the image to blob
			blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
			net.setInput(blob)

			# Outs is an array that contains all the information about objects detected,
			# their position and the confidence about the detection
			outs = net.forward(output_layers)

			# Showing information on the screen
			# initializing arrays to hold the information
			class_ids = []
			confidences = []
			boxes = []

			# iteration in the outs array, which contain the detection info
			for out in outs:
				for detection in out:
					# getting the part which contain the scores
					# from 5 to the end of the array
					scores = detection[5:]
					class_id = np.argmax(scores)
					confidence = scores[class_id]
					# if the confidence higher than 80%
					if confidence > 0.8:
						# Object detected
						# detection array hold the position of the object
						# [center_x,center_y, w, h]
						# we multiply with original image width and height
						center_x = int(detection[0] * width)
						center_y = int(detection[1] * height)

						w = int(detection[2] * width)
						h = int(detection[3] * height)

						# Rectangle coordinates
						x = int(center_x - w / 2)
						y = int(center_y - h / 2)


						"""				 Reaction on the detected objects							 """
						# some testing code for reactions
						# first we detect in which side of the road is the objects


						"""            if the object close and on the right, left, center            """

						# then if it is close and on the right
						if (  (center_y + h/2)  > height/3) and (center_x >= 2*width/3):
							# get the type of the object
							object_detected = classes[class_id]
							# print type and location
							print("{} detected close on the right".format(classes[class_id]))
							# taking actions:


							# 1) first if its a cross_walk we slow down the speed to 0.2
							if object_detected =='cross_walk':
								print("slowing down to speed 0.2")
								# send commands to the car and send the same steering but
								# different speed
								last_excution_cross_walk = True
								if lock == 'free':
									cross_walk_thread = threading.Thread( target=cross_walk_reaction, )
									cross_walk_thread.start()



							# 2)  second if its a stop_sign we stop
							elif object_detected =='stop_sign':
								print("stop the car for 3 seconds")
								# send commands to the car and send the same steering but
								stop_handle_thread = threading.Thread( target=stop_reaction, args=(last_excution_time,))
								stop_handle_thread.start()
								last_excution_time = round(time.time(), 2)



							# 3) if its a high_way we increase speed
							elif object_detected =='high_way':
								# turn on overpassing
								allow_car_passing = True
								print("increasing the speed to 0.6")
								# send commands to the car and send the same steering but
								car.drive( 0.6 , get_current_steer() )

							# 4) fourth if its a high_way wnd  we stop
							elif object_detected =='end_high_way':
								# turn off overpassing
								allow_car_passing = False
								print("decreasing the speed to 0.4")
								# send commands to the car and send the same steering but
								car.drive( 0.4 , get_current_steer() )

							# 5) fifth if its a parking sign
							elif object_detected =='parking_sign':
								if manual_turn_command == 'parking':
									# turn on parking mode if there is a spot
									parking_mode = True
									print("searching for an empty spot to park")

							# 6) if we detect a spot and parking mode is on
							elif object_detected == 'parking_spot_vertical' and parking_mode == True:
								# turning parking procedures
								# send commands to the car and send the same steering
								print("Decreasing the speed down to 0.2")
								car.drive( 0.2 , get_current_steer() )
								# switching prediction from the parking CNN model
								driving_mode = 7

							# 7) if its a traffic light
							elif object_detected == "traffic_light":
								# determine which traffic light of the four based on the closest distance
								# test the traffic light module
								traffic_light_thread = threading.Thread(target=traffic_light_reaction, args=(car_postion,))
								traffic_light_thread.start()







						# if the object close and on the left side
						elif ( (center_y + h/2) > height/3) and (center_x <= width/3):
							print("{} detected close on the left".format(classes[class_id]))
							# person on left and very close
							if object_detected =='person':
								if (center_y + h/2) > (2 * height/3):
									print("person ahead left, stop the car")
									# send commands to the car and send the same steering but
									# make a thread  to deal with pedistrian
									last_excution_person_stop = True
									# run a thread to deal this situation
									pedestrian_handle_thread = threading.Thread( target=pedestrian_reaction,)
									pedestrian_handle_thread.start()



						# if the object close and on the center
						elif ( (center_y + h/2) > height/3) and ( (center_x > width/3) and (center_x < 2*width/3)):
							# get the type of the object
							object_detected = classes[class_id]
							print("{} detected  on the center".format(classes[class_id]))
							# taking actions:


							# 1) first if its a person we stop the car
							if object_detected =='person':
								if (center_y + h/2) > (2 * height/3):
									print("person ahead, stop the car")
									# send commands to the car and send the same steering but
									# make a thread  to deal with pedistrian
									last_excution_person_stop = True
									# run a thread to deal this situation
									pedestrian_handle_thread = threading.Thread( target=pedestrian_reaction,)
									pedestrian_handle_thread.start()





							# 2) first if its a intersection we stop the car
							elif object_detected =='intersection':
								# when the intersection is very close
								if (center_y + h/2) > (2 * height/3):
									print("intersection is very close")

									# calculate the disitance from the final
									# some basic calculations to figure out the distance
									# between the current car location and the current point
									v1 = (x_dis - x) ** 2
									v2 = (y_dis - y) ** 2
									distinatin_distance = sqrt(v1 + v2)
									# calculate the slope between the current postion and the distination
									slope = (y - y_dis) / (x - x_dis)
									if ((3.2 > yaw > 0) and (slope < 0)) or ( (-3.2 < yaw < 0) and (slope > 0)):
										print("distination is on the right")
										# send commands to the car from the turn right model
										driving_mode = 1

									elif (3.2 > yaw > 0) and (slope > 0) or ( (-3.2 < yaw < 0) and (slope < 0)):
										print("distination is on the left")
										print("make a left turn")
										# send commands to the car from the turn left model
										driving_mode = 2

									#
									# # make side decision manual for now
									# if manual_turn_command == 'right':
									# 	print("make a right turn")
									# 	# send commands to the car from the turn right model
									# 	driving_mode = 1
									#
									# elif manual_turn_command == 'left':
									# 	print("make a left turn")
									# 	# send commands to the car from the turn left model
									# 	driving_mode = 2
									#
									# elif manual_turn_command == 'go ahead':
									# 	print("make a go ahead turn")
									# 	# send commands to the car from the go ahead model
									# 	driving_mode = 3


							# 3) if its a car
							elif object_detected =='car':
								# 1) very close car and not a highway and not parking mode
								if ((center_y + h/2) > (2 * height/3) and (allow_car_passing == False)) and (driving_mode != 7):
									print("car ahead very close, stop the car and tail the car")
									# send commands to the car and send the same steering but
									# different speed
									last_excution_tail = True
									tail_stop_thread = threading.Thread(target=tail_stop_reaction)
									tail_stop_thread.start()


							# 4) if it a cross walk
							elif object_detected =='cross_walk_road':
								# when the cross walk road is close
								last_excution_cross_walk = True
								if lock == 'free':
									cross_walk_thread = threading.Thread( target=cross_walk_reaction, )
									cross_walk_thread.start()




						# making a list to hold all rectangles
						boxes.append([x, y, w, h])
						# making a list to hold all confidences
						confidences.append(float(confidence))

						# making a list to hold all classes
						class_ids.append(class_id)



			""" drawing rectangles on objects after detecting """
			# # to join multiple objects which in real the are only one
			# indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
			# # declare the font type to write on the images
			# font = cv2.FONT_HERSHEY_PLAIN
			# # some instructions to write on the image
			# for i in range(len(boxes)):
			# 	if i in indexes:
			# 		x, y, w, h = boxes[i]
			# 		label = str(classes[class_ids[i]])
			# 		color = [ 0,255,0 ]
			# 		relative_pos = int( h * 1.3 )
			# 		label = label + " " + str(round(confidences[i],2))
			# 		cv2.rectangle(img,(x,y),(x+w,y+h),color,1)
			# 		cv2.putText(img, label, (x, y + relative_pos), font, 1, color, 1)




		"""									 Showing the frames 					"""
		# show the image on the previw
		cv2.imshow("Frame preview", img)
		key = cv2.waitKey(1)

		# the loop will produce 100 frames per seconds
		limit_frames += 1
		# sleep(0.01)

		""" 						buttons to make some manual actions				 """
		# to exit
		if key == ord('q'):
			cv2.destroyAllWindows()
			# stop the car
			car.drive(0.0, 0.0)
			break

		# to stop recording press p
		if key == ord('p'):
			print("recording stopped")
			record = False
		# to stop recording press r
		if key == ord('r'):
			print("recording resummed")
			record = True

		if key == ord(';'):
			manual_turn_command = 'right'
			print("upcoming turn is right")
		if key == ord('l'):
			manual_turn_command = 'left'
			print("upcoming turn is left")
		if key == ord('h'):
			manual_turn_command = 'go ahead'
			print("upcoming go ahead")
		if key == ord('t'):
			manual_turn_command = 'take over'
			driving_mode = 4
			print("upcoming take over")
		if key == ord('z'):
			manual_turn_command = 'parking'
			print("make a parking ")
		if key == ord('m'):
			manual_driving = True
			print("manual driving is on")
		if key == ord('o'):
			manual_driving = False
			print("manual driving is off")



	# when finished and the end of the thread
	print("car has been stopped \n END")
	# # turning the list to dictinary becasue its easy to deal with
	# all_data = { 'Im_paths': image_paths, 'steering': steering_list }
	# making a csv file to hold paths and steering DataSet
	with open('DataSet/driving_log.csv','ab') as f:
		writer = csv.writer(f)
		# titles
		writer.writerow(['Im_paths','steering'])
		# loop to save all recorded values
		for counter in range( len(image_paths) ):
			writer.writerow( [image_paths[counter], steering_list[counter]] )
	# when finished and the end of the thread
	print("Dataset Has been saved. \n END")





""" 									manual controling thread										"""

# creating a thread to show and save images with the related steering
# angles on parallel with the control command
thread_img = threading.Thread( target=images_thread )
thread_img.start()

# main Thread : control thread

# defining initialize variables
steering = 0.0
speed = 0.0

max_speed = 2
max_steering = 24

speed_step = 0.05
steering_step = 1


# new code for controlling the Car
def on_press(key):
	# when pressing, show what charter you pressed
	# print('{0} pressed'.format(key))
	# make speed and steering global variables
	# to save last changes every time we call the function
	global speed
	global steering

	# when pressed ESC
	if key == Key.esc:
		print("End control commands")
		# Stop listener
		return False

	# if we press w
	if key.char == "w":
		# when we reach the max speed which is 10
		# we will not increase the value of speed more
		# because even if we did, there will be no changes
		# on the real car
		if get_current_speed() < max_speed:
			# increase the speed by 0.2
			speed = get_current_speed() + speed_step
			# send commands to the car
			car.drive( speed, get_current_steer())

	# if we press s
	if key.char == "s":
		if get_current_speed() > -max_speed:
			# decrease the speed by 0.2
			speed = get_current_speed() - speed_step
			# send commands to the car
			car.drive( speed, get_current_steer())

	# to steer right
	if key.char == "d":
		# when we reach the max steering which is 25
		# we will not increase the value of steering more
		# because even if we did, there will be no changes
		# on the real car
		if get_current_steer() < max_steering:
			steering = get_current_steer() + steering_step
			# send commands to the car
			car.drive( get_current_speed(), steering)

	# to steer left
	if key.char == "a":
		if get_current_steer() > -max_steering:
			steering = get_current_steer() - steering_step
			# send commands to the car
			car.drive( get_current_speed(), steering)

	# to show the steer and speed on the screen
	print('speed {0}, steering {1}'.format( get_current_speed(), get_current_steer()) )


# maybe modified later
def on_release(key):
	pass

# Collect events until released
with Listener( on_press=on_press, on_release=on_release) as listener:
	listener.join()
