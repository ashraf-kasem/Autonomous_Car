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
import bfmclib.controller_p
import pickle
import numpy as np
import gzip


import threading
import rospy
import cv2
from time import sleep
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

# creating a BNO object
bno = BNO055()
print("Bno created")


# create a GPS object
gps = Gps()
print(" Gps created ")

# create TrafficLight object
traffic_light = TrafficLight()
print(" traffic lights  created ")

# if nyou want to exit frame preview
print("Select \"Frame preview\" window and press Q to exit")
print("Select \"Frame preview\" window and press P to stop the recording")
print("Select \"Frame preview\" window and press R to resume the recording")




# function to be multithreaded with the control thread
def images_thread():

	# defining list to hold all frames and related steering angles
	image_paths = []
	steering_list = []

	# a variable to hold the recording status
	record = False

	# to name the images
	im_number = 0

	# counter to limit recording to 10 frame per second
	limit_frames = 0

	while True:


		# if limit_frames % 40 == 0:


			# testing some BNO methods

			# not usful
			# print("oz is ", bno.getOz())
			# #
			# # not usful
			# print("Roll is ",bno.getRoll())
			#
			# when there is a ramp
			# print("Pitch",bno.getPitch())
			#
			# # between 0 to 3 and -3 to 0
			# print("Yaw ", bno.getYaw())



			# # test the GPS methods
			# gps_data_dic = gps.getGpsData()
			# time_in_seconds = gps_data_dic['timestamp']
			# coordinations = gps_data_dic['coor']
			#
			# # process data
			# if coordinations:
			# 	# get postion on x and y axis
			# 	x = coordinations[0].real
			# 	y = coordinations[0].imag
			#
			# 	# git orentation in comlex form
			# 	# range [-1 to 1] + [ -1 t0 1] j
			# 	rotation = coordinations[1]
			#
			# 	# print info
			# 	print("postion x:" ,x, " y:", y )
			# 	print("rotation" ,rotation )
			#
			# # to get nuber of seconds passed from starting the simulation
			# if time_in_seconds:
			# 	print("seconds is passed from the simulator starts is" ,time_in_seconds )
			#
			#
			#
			#
			#
			#
			# # test the traffic light module
			# # 0 means red, 1 means yellow, 2 means green
			# print("traffic light placed at the starting point ",traffic_light.getTL0State())
			# print("for the traffic light placed in the WEST ",traffic_light.getTL1State())
			# print("for the traffic light placed in the SOUTH",traffic_light.getTL2State())
			# print("for the traffic light placed in the EAST ",traffic_light.getTL3State())



		# by default it is False
		if record :

			# to record 10 fps
			if limit_frames % 10 == 0:

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

		# show the image on the previw
		cv2.imshow("Frame preview", cam.getImage())
		key = cv2.waitKey(1)


		# the loop will preduce 100 frames per seconds
		limit_frames += 1
		sleep(0.01)

		# to exit
		if key == ord('q'):
			cv2.destroyAllWindows()
			# stop the car
			car.drive(0.0, 0.0)
			print("Waiting for the data set to be saved... dont close !!!")
			break

		# to stop recording press p
		if key == ord('p'):
			print("recording stopped")
			record = False

		# to stop recording press r
		if key == ord('r'):
			print("recording resummed")
			record = True



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


	# # # # open a pickle file to save data into
	# with gzip.open("Training_data.pklz", "wb") as f:
	# 	pickle.dump(all_data, f)

	# when finished and the end of the thread
	print("Dataset Has been saved. \n END")


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

speed_step = 0.05
steering_step = 1


# new code for controlling the Car
def on_press(key):
	# when pressing, show what charchter you pressed
	print('{0} pressed'.format(key))
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
		if speed < max_speed:
			# increase the speed by 0.5
			speed += speed_step

	# if we press s
	if key.char == "s":
		if speed > -max_speed:
			# decrease the speed by 0.5
			speed -= speed_step

	# to steer right
	if key.char == "d":
		# when we reach the max steering which is 25
		# we will not increase the value of steering more
		# because even if we did, there will be no changes
		# on the real car
		if steering < max_steering:
			steering += steering_step

	# to steer left
	if key.char == "a":
		if steering > -max_steering:
			steering -= steering_step



	# send commands to the car
	car.drive(speed, steering)
	print('speed {0}, steering {1}'.format(speed, steering))


# maybe modified later
def on_release(key):
	# print('{0} release'.format(key))
	# global speed
	# global steering
	# # if key.char == "w":
	# # 	speed += 0.5
	# # if key.char == "s":
	# # 	speed -= 0.5
	# # if key.char == "a":
	# # 	steering = 0
	# # if key.char == "d":
	# # 	steering = 0
	# car.drive(speed, steering)
	# print('speed {0}, steering {1}'.format(speed, steering))
	pass
	# sending commands to the car

# Collect events until released
with Listener( on_press=on_press, on_release=on_release) as listener:
	listener.join()
