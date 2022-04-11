#!/usr/bin/python

"""
These are the libraries you need to import in your project in order to
be able to communicate with the Gazebo simulator
"""
#Only for use in Python 2.6.0a2 and later
from __future__ import print_function

from bfmclib.gps_s import Gps
from bfmclib.bno055_s import BNO055
from bfmclib.camera_s import CameraHandler
from bfmclib.controller_p import Controller
from bfmclib.trafficlight_s import TLColor, TLLabel, TrafficLight
import bfmclib.controller_p
import pickle
import numpy as np
import gzip

from math import sqrt
import bs4 as bs
import threading
import rospy
import cv2
from time import sleep
from pynput.keyboard import Key, Listener
from datetime import datetime
from PIL import Image
import os
import csv


# Enter the distination coordinations
dis = raw_input("please Enter the distination coordinates: ex 3.5 1.2 \n ")
x_y =  dis.split()
x_dis = eval( x_y[0] )
y_dis = eval( x_y[1] )


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

# show the dictinary format
# print(nodes_dict)

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

# print the edge dict to show the format
# print(edges_dict)



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


		if limit_frames % 10 == 0:


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
			# between 0 to 3 and -3 to 0
			yaw = bno.getYaw()
			print("Yaw ",yaw)



			# test the GPS methods
			gps_data_dic = gps.getGpsData()
			# time_in_seconds = gps_data_dic['timestamp']
			coordinations = gps_data_dic['coor']

			# process data
			if coordinations:
				# get postion on x and y axis
				x = coordinations[0].real
				y = coordinations[0].imag

				# git orentation in comlex form
				# range [-1 to 1] + [ -1 t0 1] j
				# rotation = coordinations[1]

				# print info
				print("postion x:" ,x, " y:", y )
				# print("rotation" ,rotation )

				# calculate the disitance from the final
				# some basic calculations to figure out the distance
				# between the current car location and the current point
				v1 = (x_dis - x) ** 2
				v2 = (y_dis - y) ** 2
				distinatin_distance = sqrt( v1 + v2 )
				# calculate the slope between the current postion and the distination
				slope = (y - y_dis)/(x - x_dis )
				if ((yaw < 3.2  and yaw > 0) and (slope < 0)) or ((yaw > -3.2  and yaw < 0) and (slope > 0)):
					print("distination is on the right")
				elif (yaw < 3.2  and yaw > 0) and (slope > 0) or ((yaw > -3.2  and yaw < 0) and (slope < 0)) :
					print("distination is on the left")

				print("the distance for the distination is : ", distinatin_distance)
				print("the slope between the current postion and the distination is: ",slope )


				# some testing to know what is the closest map point to car postion
				# define a start value to be the minmal distance
				min_distance = 1000
				# iterate over all items of the nodes
				for (key,value) in nodes_dict.items():
					# some basic calculations to figure out the distance
					# between the current car location and the current point
					v1 = (value[0] - x) ** 2
					v2 = (value[1] - y) ** 2
					current_distance = sqrt( v1 + v2 )
					# save the key of the point that has the minmal distance value
					if current_distance < min_distance:
						id = key
						min_distance = current_distance

				# show the info on the termainal about the closest point
				print("the closest node is node number: {}, coor: {}".format( id, nodes_dict[id] ))
				print("Distance is : {}".format(min_distance))



				# # to get nuber of seconds passed from starting the simulation
				# if time_in_seconds:
				# 	print("seconds is passed from the simulator starts is" ,time_in_seconds )

			# test the traffic light module
			# 0 means red, 1 means yellow, 2 means green
			print("traffic light placed at the starting point ",traffic_light.getTL0State())
			print("for the traffic light placed in the WEST ",traffic_light.getTL1State())
			print("for the traffic light placed in the SOUTH",traffic_light.getTL2State())
			print("for the traffic light placed in the EAST ",traffic_light.getTL3State())

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

speed_step = 0.2
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
