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
import numpy as np
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
# build object from the car controller
car = Controller()
print("Controller loaded")

# if you want to exit frame preview
print("Select \"Frame preview\" window and press Q to exit")
print("Select \"Frame preview\" window and press P to stop the recording")
print("Select \"Frame preview\" window and press R to resume the recording")



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
	model_change_lane_right = load_model('model_keep_for_change_lane_right_aug_150.h5')



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

			if turn_counter > 25:
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

			if turn_counter > 50:
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

			if turn_counter > 25:
				# return to left lane keeping mode
				# return the manual turn command to null
				manual_turn_command = ''
				driving_mode = 0
			# predict the steering angles
			# reused the lane keeping for going ahead on intersection
			pred_steering = float(model_change_lane_right.predict(image))
			print("change lane right")
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


						"""            if the object closed and on the right, left, center            """

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
								car.drive( 0.2 , get_current_steer() )



							# 2)  second if its a stop_sign we stop
							elif object_detected =='stop_sign':
								print("stop the car ")
								# send commands to the car and send the same steering but
								# car.drive( 0.0 , get_current_steer() )


							# 3) if its a high_way we increase speed
							elif object_detected =='high_way':
								# turn on overpassing
								allow_car_passing = True
								print("increasing the speed to 0.6")
								# send commands to the car and send the same steering but
								car.drive( 0.6 , get_current_steer() )

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


							# 1) first if its a person we stop the car
							if object_detected =='person':
								if (center_y + h/2) > (2 * height/3):
									print("person ahead, stop the car")
									# send commands to the car and send the same steering but
									# different speed
									car.drive( 0.0, get_current_steer() )

							# 2) first if its a intersection we stop the car
							elif object_detected =='intersection':
								# when the intersection is very close
								if (center_y + h/2) > (2 * height/3):
									print("intersection is very close")

									# make side decision manual for now
									if manual_turn_command == 'right':
										print("make a right turn")
										# send commands to the car from the turn right model
										driving_mode = 1

									elif manual_turn_command == 'left':
										print("make a left turn")
										# send commands to the car from the turn left model
										driving_mode = 2

									elif manual_turn_command == 'go ahead':
										print("make a go ahead turn")
										# send commands to the car from the go ahead model
										driving_mode = 3


							# 3) if its a car
							elif object_detected =='car':
								# 1) very close car and not a highway
								if (center_y + h/2) > (2 * height/3) and (allow_car_passing == False):
									print("car ahead very close, stop the car")
									# send commands to the car and send the same steering but
									# different speed
									car.drive( 0.0 , get_current_steer() )



						# making a list to hold all rectangles
						boxes.append([x, y, w, h])
						# making a list to hold all confidences
						confidences.append(float(confidence))

						# making a list to hold all classes
						class_ids.append(class_id)



			# """ drawing rectangles on objects after detecting """
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

speed_step = 0.2
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
