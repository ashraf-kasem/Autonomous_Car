#!/usr/bin/python

"""
These are the libraries you need to import in your project in order to
be able to communicate with the Gazebo simulator
"""
from bfmclib.gps_s import Gps
from bfmclib.bno055_s import BNO055
from bfmclib.camera_s import CameraHandler
from bfmclib.controller_p import Controller
from bfmclib.trafficlight_s import TLColor, TLLabel, TrafficLight
from bfmclib.controller_p import get_current_steer, get_current_speed
import bfmclib.controller_p
import pickle
import numpy as np
import gzip


import threading
import rospy
import cv2
from time import sleep
from keras.models import load_model
from pynput.keyboard import Key, Listener
from datetime import datetime
from PIL import Image
import os
import csv


# This line should be the first line in your program
rospy.init_node('main_node', anonymous=True)

# build object from the car handler
cam = CameraHandler()
print("Camera loaded")

car = Controller()
print("Controller loaded")

# if nyou want to exit frame preview
print("Select \"Frame preview\" window and press Q to exit")
print("Select \"Frame preview\" window and press P to stop the recording")
print("Select \"Frame preview\" window and press R to resume the recording")





# to preprocess the image before making a prediction
def img_preprocess(img):
	img = img[150:,:,:]
	img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
	img = cv2.GaussianBlur(img,  (3, 3), 0)
	img = cv2.resize(img, (200, 66))
	img = img/255
	img = img.reshape(66, 200, 1)
	return img




# function to be multithreaded with the control thread
def images_thread():

	# load the lane keeping model
	model = load_model('model_lane_keeping.h5')

	# defining list to hold all frames and related steering angles
	image_paths = []
	steering_list = []


	# counter to limit  frame per second when using the model
	# to not over helm the processor
	limit_frames = 0

	# a variable to allow car ver passing on high way
	allow_car_passing = False

	# Load Yolo trained model for traffic detection and its configuration
	# we are using opencv library version 4
	net = cv2.dnn.readNet("yolov3-tiny_final.weights", "yolov3-tiny.cfg")

	# load the classes for the traffic detection from classes.names file
	classes = []
	with open("classes.names", "r") as f:
	    classes = [line.strip() for line in f.readlines()]

	# some config for the detection model
	# we want to gwt the output of the last layers
	layer_names = net.getLayerNames()
	output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]



	while True:

		# Loading image
		img =  cam.getImage()

		# preprocess the current frame
		image = img_preprocess(img)
		image = np.array([image])

		# using the olane keeping model to predict
		pred_steering = float(model.predict(image))

		# detection part
		# to reduce the overhead on the cpu
		# we will only apply the detection on 2 frames perseconds
		if limit_frames :

			# sending commands based on the lane keeping model
			car.drive( get_current_speed() , pred_steering )
			print(get_current_speed() , get_current_steer() )

			# grapping the peoprties of the image
			height, width, channels = img.shape
			# Detecting objects
			#  first we need to convert the image to blob
			blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
			net.setInput(blob)

			# Outs is an array that conains all the informations about objects detected,
			# their position and the confidence about the detection
			outs = net.forward(output_layers)

			# Showing informations on the screen
			# intilizing arraies to hold the information
			class_ids = []
			confidences = []
			boxes = []

			# iteration in the outs array, which contain the detection info
			for out in outs:
			    for detection in out:
					# getting the part whch contain the scores
					# from 5 to the end of the array
					scores = detection[5:]
					class_id = np.argmax(scores)
					confidence = scores[class_id]
					# if the confidence higher than 80%
					if confidence > 0.8:
						# Object detected
						# detection array hold the postion of the object
						# [center_x,center_y, w, h]
						# we multiply with orginal image width and height
						center_x = int(detection[0] * width)
						center_y = int(detection[1] * height)

						w = int(detection[2] * width)
						h = int(detection[3] * height)

						# Rectangle coordinates
						x = int(center_x - w / 2)
						y = int(center_y - h / 2)


						# some testing code for ractions
						# first we detecte in which side of the road is the objects
						# then if it is close and on the right
						if ( (center_y + h/2)   > height/3) and (center_x >= 2*width/3):
							# get the type of the object
							object_detected = classes[class_id]
							# print type and location
							print("{} detected close on the right".format(classes[class_id]))

							# taking actions:

							# first if its a cross_walk we slow down the speed to 0.2
							if object_detected =='cross_walk':

								print("slowing dwon to speed 0.2")
								# send commands to the car and send the same steering but
								# diffrent speed
								car.drive( 0.2 , get_current_steer() )

							# second if its a stop_sign we stop
							elif object_detected =='stop_sign':

								print("stop the car ")
								# send commands to the car and send the same steering but
								car.drive( 0.0 , get_current_steer() )

							# third if its a high_way we increase speed
							elif object_detected =='high_way':
								# turn on overpassing
								allow_car_passing = True
								print("increasing the speed to 1")
								# send commands to the car and send the same steering but
								car.drive( 1 , get_current_steer() )

							# fourth if its a high_way wnd  we stop
							elif object_detected =='end_high_way':
								# turn off overpassing
								allow_car_passing = False
								print("decreasing the speed to 0.4")
								# send commands to the car and send the same steering but
								car.drive( 0.4 , get_current_steer() )


						# if the object close and on the left side
						elif ( (center_y + h/2) > height/3) and (center_x <= width/3):
							print("{} detected close on the left".format(classes[class_id]))

						# if the object close and on the center
						elif ( (center_y + h/2) > height/3) and ( (center_x > width/3) and (center_x < 2*width/3)):
							# get the type of the object
							object_detected = classes[class_id]
							print("{} detected  on the center".format(classes[class_id]))
							# taking actions:
							# first if its a person we stop the car
							if object_detected =='person':
								print("person ahead, stop the car")
								# send commands to the car and send the same steering but
								# diffrent speed
								car.drive( 0.0 , get_current_steer() )

							elif object_detected =='car':
								if (center_y + h/2) > (2 * height/3) and (allow_car_passing == False):
									print("car ahead very close, stop the car")
									# send commands to the car and send the same steering but
									# diffrent speed
									car.drive( 0.0 , get_current_steer() )

								elif (center_y + h/2) > ( height/2) and (allow_car_passing == True):
									print("car ahead in high way, overpass the car")
									# send commands to the car and send the same steering but
									# diffrent speed
									# car.drive( 0.0 , get_current_steer() )

						# making a list to hold all rectangles
						boxes.append([x, y, w, h])
						# making a list to hold all confidences
						confidences.append(float(confidence))

						# making a list to hold all classes
						class_ids.append(class_id)

			# to join multible objects which in real the are only one
			indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

			# declare the font type to write on the images
			font = cv2.FONT_HERSHEY_PLAIN

			# some instructions to write on the image
			for i in range(len(boxes)):
			    if i in indexes:
					x, y, w, h = boxes[i]
					label = str(classes[class_ids[i]])
					color = [ 0,255,0 ]
					relative_pos = int( h * 1.3 )
					label = label + " " + str(round(confidences[i],2))
					cv2.rectangle(img,(x,y),(x+w,y+h),color,1)
					cv2.putText(img, label, (x, y + relative_pos), font, 1, color, 1)


		# show the image on the previw
		cv2.imshow("Frame preview", img)
		key = cv2.waitKey(1)


		# the loop will preduce 100 frames per seconds
		limit_frames += 1
		sleep(0.01)

		# to exit
		if key == ord('q'):
			cv2.destroyAllWindows()
			# stop the car
			car.drive(0.0, 0.0)
			break



	# when finished and the end of the thread
	print("car has been stopped \n END")



# creating a thread to show and save images with the related steering
# angles on parrarel with the control command
thread_img = threading.Thread( target=images_thread )
thread_img.start()

# main Thread : control thread

# defining intilaze variables
steering = 0.0
speed = 0.0

max_speed = 10
max_steering = 24

speed_step = 0.2
steering_step = 2


# new code for controlling the Car
def on_press(key):
	# when pressing, show what charchter you pressed
	# print('{0} pressed'.format(key))
	# make soeed and steering global variables
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
