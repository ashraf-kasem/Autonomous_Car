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
import time
from keras.models import load_model
from pynput.keyboard import Key, Listener
from datetime import datetime
from math import sqrt, hypot
import os
import csv
import socket


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


# enter the destination coordinates
destination = raw_input("Pleas enter the destination coordinates:\nUse this format for x and y: 3.5 4.5\n")
destination = destination.split()
destination_x = eval(destination[0])
destination_y = eval(destination[1])

# defining a TCP socket to send car position to local server running path planing
# based on the home made xml map
host = socket.gethostname()
port = 8000

# starting connection to the path planing server
client_socket = socket.socket()
client_socket.connect((host, port))


# location of the 4 traffic signs based on the given map
# TL0 = [1 , 14.5] # TL1 = [2 , 11.2] # TL2 = [3.7,10.3] # TL3 = [3.3,11.5]
traffic_light_locations = {'TL0': [1, 14.5], 'TL1': [2, 11.2],
                           'TL3': [3.7, 10.3], 'TL2': [3.3, 11.5]}

# some global varible to deal with person reaction
last_excution_person_stop = True
last_excution_person_stop_time = 0

# some global varible to deal with cross walk reaction
last_excution_cross_walk = True
last_excution_cross_walk_time = 0

# some global varible to deal with tail close car reaction
last_excution_tail = True
last_excution_tail_time = 0

# global varibale for stop sign
last_excution_time = 0

# define a variable as a lock
lock = "free"

# define a global variable to control changing lane
first_yaw_overtake = True

# first time exit round road detected
first_time_exit_round_detected = True

# second time exit
second_time_exit_round_detected = True



"""                                 Load the xml  map and process it                                """

# open the map in xml format
map_path = os.getcwd() + "/map/map.xml"
xml_file = open(map_path, 'r')
# building a soup object to parse
map = bs.BeautifulSoup(xml_file, 'xml')
# extracting all nodes tags and build a dictinary contain all nodes id with
# its related coordination
nodes_dict = {}
for node in map.find_all('node'):
    # get all data tags from the node tag
    x, y = node.find_all('data')
    # get the value of each data tag
    x = eval(str(x.text))
    y = eval(str(y.text))
    # get the id of the each node
    id = eval(str(node['id']))
    # add element to the dictinary contain  { key = id : value = [x,y] }
    nodes_dict[id] = [x, y]
# show the dictinary format
# print(nodes_dict)
# extracting all edges tags and build a dictinary contain all edges and
# the situation of ech edge
dashed_edges = []
edges_dict = {}
for edge in map.find_all('edge'):
    # get  the value of data tag from the edge tag
    status = edge.data.text
    status = str(status)
    # get the value of source and target attributes in the edge tag
    source = eval(str(edge['source']))
    target = eval(str(edge['target']))
    # add element to the dictinary contain  { key = (source,target) : value = status }
    edges_dict[(source, target)] = status
    # get just true status ( dashed )
    if status == 'True':
        dashed_edges.extend([source, target])
# print the edge dict to show the format
# print(edges_dict)
# delete all repeated values
dashed_edges = list(set(dashed_edges))

# if you want to exit frame preview
print("Select \"Frame preview\" window and press Q to exit")
print("Select \"Frame preview\" window and press P to stop the recording")
print("Select \"Frame preview\" window and press R to resume the recording")

"""                       Define the main preprocess function               """


# to preprocess the image before making a prediction
def img_preprocess(img, driving_mode):
    # if the mode is turning left and right and change lane
    if driving_mode in [1, 2, 4]:
        img = img[120:, :, :]
    # keep lane left mode
    elif driving_mode == 5:
        img = img[150:, 250:, :]
    # parking mode
    elif driving_mode == 7:
        img = img[200:, 250:, :]
    # other modes
    else:
        img = img[150:, :, :]
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    img = cv2.GaussianBlur(img, (3, 3), 0)
    img = cv2.resize(img, (200, 66))
    # show the image on the previw
    # cv2.imshow("processed image ", img)
    # key = cv2.waitKey(1)
    # if key == ord('c'):
    #     cv2.destroyAllWindows()
    img = img / 255
    img = img.reshape(66, 200, 1)
    return img




# function to get the current postion of the car
def get_car_pos():
    # test the GPS methods
    gps_data_dic = gps.getGpsData()
    coordination = gps_data_dic['coor']
    # process data to get car postion
    # if data exist
    if coordination:
        # get postion on x and y axis
        x = coordination[0].real
        y = coordination[0].imag
        car_postion = [x, y]
        return car_postion
    # if data not exist, usually at the first couple of seconds
    else:
        return None





# thread to send the current car position and receive the path after planed
def path_planing():
    time.sleep(40)
    # first the last message will be empty
    last_message = ''
    # build a message with format "x y" for the destination,
    #  this is the first message
    message = str(destination_x) +" "+ str(destination_y)
    # send message
    client_socket.send(message.encode())
    # first message from server is HI, so its not important now
    data = client_socket.recv(1024).decode()  # receive response
    # main TCP socket loop
    while True:
        if last_message == message:
            time.sleep(1)
        # save the last message sent
        last_message = message
        # get the gps data using the following function
        car_postion = get_car_pos()
        if car_postion:
            # build a message with format "x y"
            message = str(car_postion[0]) + " " + str(car_postion[1])
        # else send a postion of zeros
        else:
            message = '0 0'
        client_socket.send(message.encode())  # send message
        data = client_socket.recv(1024).decode()  # receive response
        route_list = data.split()
        # print(route_list)

    client_socket.close()  # close the connection






# stop sign hanlding method thread
def stop_reaction():
    # get current time in seconds to compared with the last excution
    global last_excution_time
    current_time = int(gps.getGpsData()['timestamp'] * 1000) % 1000
    diffrence = current_time - last_excution_time
    # if there is a more than 3 simulatin seconds between the last execution and the this one
    if diffrence > 3:
        # save the previos speed
        previous_speed = get_current_speed()
        # make the care stop for 3 simulation seconds
        time_after_3_sec = current_time + 3
        # local variblae to count 3 simulation seconds
        exact_time = 0
        print("car stopped for 3 simulation seconds, time is from the GPS. waiting ...")
        while exact_time < time_after_3_sec:
            car.drive(0.0, get_current_steer())
            exact_time = int(gps.getGpsData()['timestamp'] * 1000) % 1000
        # return the previous_speed to the car
        car.drive(previous_speed, get_current_steer())


# traffic light reaction thread
def traffic_light_reaction(car_postion):
    # 0 means red, 1 means yellow, 2 means green
    # some testing to know what is the closest traffic point to car postion
    # define a start value to be the minmal distance
    min_distance = 1000
    traffic_id = 'TL0'
    # iterate over all items of the traffic lights
    for (key, value) in traffic_light_locations.items():
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
    # print( traffic_id,"status: ", traffic_status)
    # wait until the green color (traffic_status=0)
    if traffic_status == 2:
        # move the car, Green
        car.drive(0.4, get_current_steer())
        print(traffic_id, "status: Green ")
    elif traffic_status == 1:
        # stop the car, yellow
        car.drive(0, get_current_steer())
        print(traffic_id, "status: Yellow ")
    else:
        # stop the car, red
        car.drive(0, get_current_steer())
        print(traffic_id, "status: Red ")


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
    model_turn_go_ahead = load_model('model_round_go_ahead_150_v7.h5')
    # load the change lane left
    model_change_lane_left = load_model('model_change_lane_left_aug_120_v6.h5')
    # to keep lane in highway left
    model_keep_lane_left = load_model('model_keep_for_left_aug_120_v15.h5')
    # load the take over
    model_change_lane_right = load_model('model_keep_for_change_lane_right_aug_150_v5.h5')
    # load park vertical model
    model_parknig_vertical = load_model('model_keep_for_parking_vertical_v2.h5')
    # load the round road model
    model_round_model = load_model('model_round_road_150_v4.h5')
    # load the round road recover
    model_round_recover = load_model('model_round_road_recover_150_v6.h5')
    # load the  cross walk pass model
    model_cross_walk_pass = load_model('model_round_go_ahead_150_v3.h5')
    # load the enter round road model
    model_enter_round = load_model('model_enter_round_150.h5')



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
    # turn_counter = 0
    # to use when making a turn decision manually
    manual_turn_command = ''
    # make the driving mode is manual at start by default
    manual_driving = True
    # make the defalt value of parking mode is off
    parking_mode = False
    # car postion default value
    car_postion = [0, 0]
    # starting postion
    starting_postion = [0, 0]
    # make these varible visiable in all these thread
    global last_excution_person_stop
    global last_excution_person_stop_time
    global last_excution_cross_walk
    global last_excution_cross_walk_time
    global last_excution_tail_time
    global last_excution_tail
    global last_excution_time
    global lock
    global first_yaw_overtake
    global first_time_exit_round_detected
    global second_time_exit_round_detected


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
        if record:
            # to record 10 fps
            if limit_frames:
                # get the exact date to combign it with image name
                exact_time = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p")
                # build the directory and the name of each image
                im_name = os.getcwd() + "/DataSet/IMG/img_{}{}.jpg".format(exact_time, im_number)
                # # using Image class fron PIL modul to get and save the image
                # im = Image.fromarray( cam.getImage() )
                # im.save(im_name)
                cv2.imwrite(im_name, cam.getImage())
                # append image by imag path
                image_paths.append(im_name)
                # increasing the image counter number, to makesure all the images
                # have diffrent names
                im_number += 1
                # images are (480, 640, 3)
                # print(cam.getImage().shape)
                # get the steering angle on every frame
                steering_list.append(bfmclib.controller_p.get_current_steer())

        # Loading image
        img = cam.getImage()
        # preprocess the current frame
        image = img_preprocess(img, driving_mode)
        image = np.array([image])

        # test the GPS methods
        gps_data_dic = gps.getGpsData()
        # # simulation time in seconds
        # current_time_in_seconds = int(gps.getGpsData()['timestamp'] * 1000) % 1000

        # between 0 to 3 and -3 to 0
        current_yaw = bno.getYaw()
        # to see if we are on a ramp or not
        current_pitch = bno.getPitch()
        if current_pitch < -0.01 :
            print("Ramp up detected ")
        elif current_pitch > 0.01:
            print("Ramp down detected ")


        coordination = gps_data_dic['coor']
        # process data to get car postion
        if coordination:
            # get postion on x and y axis
            x = coordination[0].real
            y = coordination[0].imag
            car_postion = [x, y]
            # starting value for calculating the lower distance for the dashed line
            distance_to_closest_dashed_line = 1000
            # loop over all dashed points to see what is the distance to closest dashed line
            for point in dashed_edges:
                # check distance to all dashed points
                current_distance_to_dashed_line = hypot(nodes_dict[point][0] - car_postion[0],
                                                        nodes_dict[point][1] - car_postion[1])
                # when thr current distance is lower
                if current_distance_to_dashed_line <= distance_to_closest_dashed_line:
                    distance_to_closest_dashed_line = current_distance_to_dashed_line
                    point_ID = point
            # print the id and the distance for the closest dashed line point
            # print(point_ID, distance_to_closest_dashed_line)
            # when the distance to the nearest dashed line point is less than 0.5 unit
            # make car over taking on
            if distance_to_closest_dashed_line < 0.5 :
                if not allow_car_passing:
                    print("dashed lines, car over take allowed")
                allow_car_passing = True
            # when when the distance to the nearest dashed line point is more than 0.5 unit
            # make car overtaking off, to not rin the command all the time
            else:
                if  allow_car_passing:
                    print("end dashed lines, car over take not allowed")
                allow_car_passing = False

        """   Choosing what AI model to use based on the Driving mode      """
        # means make a turn right
        if driving_mode == 1:
            # using the turn right model to predict
            # print( abs( current_yaw - last_yaw) )
            if (current_yaw > 1.5) and (last_yaw < -1.5):
                # one situation to be over come
                last_yaw = last_yaw + 6.3
            if abs(current_yaw - last_yaw) > 1.35:
                # return to lane keeping mode
                # return the manual turn command to null
                manual_turn_command = ''
                driving_mode = 0
            # predict the steering angles
            pred_steering = float(model_turn_right.predict(image))
            print("Turn right is active")


        # means a turn left
        elif driving_mode == 2:
            # using the turn right model to predict
            # print( abs( current_yaw - last_yaw) )
            if (last_yaw > 1.5) and (current_yaw < -1.5):
                # one situation to be over come
                current_yaw = current_yaw + 6.3
            if abs(current_yaw - last_yaw) > 1.1:
                # return to lane keeping mode
                # return the manual turn command to null
                manual_turn_command = ''
                driving_mode = 0
            # predict the steering angles
            pred_steering = float(model_turn_left.predict(image))
            print("Turn left is active")



        # means make go ahead turn
        elif driving_mode == 3:
            # using the go ahead model to predict
            # calculating the postion diffrence to be at least 1 unit
            diffrenc_postion = hypot(starting_postion[0] - car_postion[0], starting_postion[1] - car_postion[1])
            # use gps time to change the lane to right after 1 or more  simulation seconds
            current_time = int(gps.getGpsData()['timestamp'] * 1000) % 1000
            diffrenc_time = current_time - start_go_ahead_time
            print('diffrenc_time', diffrenc_time)
            print('diffrenc_postion', diffrenc_postion)
            if (diffrenc_time >= 1) and (diffrenc_postion >= 0.7):
                # return to lane keeping mode
                # return the manual turn command to null
                manual_turn_command = ''
                driving_mode = 0
            # predict the steering angles
            # reused the lane keeping for going ahead on intersection
            pred_steering = float(model_turn_go_ahead.predict(image))
            print("Go ahead is active")



        # means make change lane left
        elif driving_mode == 4:
            # to save just first Yaw value
            first_yaw_overtake = False
            # using the go ahead model to predict
            # when the counter reaches 100 the car will finish the go ahead mode
            # print(abs(current_yaw - last_yaw))
            # print(last_yaw)
            # print(current_yaw)
            # to solve 3,-3 situation
            if (last_yaw > 1.5) and (current_yaw < -1.3):
                # one situation to be over come
                current_yaw = current_yaw + 6.3
            if abs(current_yaw - last_yaw) > 0.3:
                # return to left lane keeping mode
                # return the manual turn command to null
                # save the moment of starting keep left lane mode
                start_keep_left_time = int(gps.getGpsData()['timestamp'] * 1000) % 1000
                # save the satrting position
                starting_postion = car_postion
                manual_turn_command = ''
                driving_mode = 5

            # predict the steering angles
            # reused the lane keeping for going ahead on intersection
            pred_steering = float(model_change_lane_left.predict(image))
            print("change lane left is active")


        # using the keep left lane model to predict
        elif driving_mode == 5:
            # using the keep left lane model to predict
            # calculating the postion diffrence to be at least 1 unit
            diffrenc_postion = hypot(starting_postion[0] - car_postion[0], starting_postion[1] - car_postion[1])
            # use gps time to change the lane to right after 1 or more  simulation seconds
            current_time = int(gps.getGpsData()['timestamp'] * 1000) % 1000
            diffrenc_time = current_time - start_keep_left_time
            # print('diffrenc_time', diffrenc_time)
            # print('diffrenc_postion', diffrenc_postion)
            if (diffrenc_time >= 1) and (diffrenc_postion >= 1):
                # return to left lane keeping mode
                # return the manual turn command to null
                # save the last orintation
                last_yaw = bno.getYaw()
                manual_turn_command = ''
                driving_mode = 6

            # predict the steering angles
            # reused the lane keeping for going ahead on intersection
            pred_steering = float(model_keep_lane_left.predict(image))
            print("keep left lane is active")



        elif driving_mode == 6:
            # using the go ahead model to predict
            # when the counter reaches 100 the car will finish the go ahead mode
            # print the distance between starting distance and current one
            print(abs(current_yaw - last_yaw))
            # to solve 3, -3 situation
            if (current_yaw > 1.5) and (last_yaw < -1.5):
                # one situation to be over come
                last_yaw = last_yaw + 6.2
            if abs(current_yaw - last_yaw) > 0.18:
                # return to left lane keeping mode
                # return the manual turn command to null
                manual_turn_command = ''
                # to make the car over take again if neccessarly
                first_yaw_overtake = True
                driving_mode = 0
            # predict the steering angles
            # reused the lane keeping for going ahead on intersection
            pred_steering = float(model_change_lane_right.predict(image))
            print("change lane right")


        # means park vertical
        elif driving_mode == 7:
            # using the turn right model to predict
			# to make sure that overtaking is off when parking
            allow_car_passing = False
            # print( abs( current_yaw - last_yaw) )
            if (current_yaw > 1.5) and (last_yaw < -1.5):
                # one situation to be over come
                last_yaw = last_yaw + 6.3
            if abs(current_yaw - last_yaw) > 1.5:
                # means parking ends
                print("parking has ended, the car has stopped")
                driving_mode = 9
            # using the park vertical model to predict
            # predict the steering angles
            pred_steering = float(model_parknig_vertical.predict(image))
            print("park vertical is active")




        # means a round mode first part
        elif driving_mode == 8:
            # using theround mode  model to predict
            # calculate thr deffrence of the current postion and the starting postion
            diffrenc_postion = hypot(starting_postion[0] - car_postion[0], starting_postion[1] - car_postion[1])
            # use gps time to change the lane to right after 1 or more  simulation seconds
            # current_time = int(gps.getGpsData()['timestamp'] * 1000) % 1000
            # diffrenc_time = current_time - starting_rotate_time
            # some output
            # print( 'diffrenc_time',diffrenc_time )
            # print( 'diffrenc_postion' , diffrenc_postion )
            # print( 'last_yaw' , last_yaw )
            # print( 'current_yaw' , current_yaw )
            # print( abs( current_yaw - last_yaw) )
            if diffrenc_postion < 0.7:
                pred_steering = float(model_enter_round.predict(image))
                print("first part if round mode is active")
            # when the condtion is not true swicthc to 	second part of the round mode
            else:
                # switch to driving mode 11, second part of the round mode
                driving_mode = 11
                # save the satrting position
                starting_postion = car_postion

        # means a round mode second part
        elif driving_mode == 11:
            pred_steering = float(model_round_model.predict(image))
            print("second part if round mode is active")


        # means a round mode third part, recovery part
        elif driving_mode == 12:
            # predict the steering angles
            # to not react more than once to the exit round road mode
            first_time_exit_round_detected = False
            second_time_exit_round_detected = True
            # calculate thr deffrence of the current postion and the starting postion
            diffrenc_postion = hypot(starting_postion[0] - car_postion[0], starting_postion[1] - car_postion[1])
            # some output
            # print( 'diffrenc_postion' , diffrenc_postion )
            # end recovering mode
            if diffrenc_postion >= 1.2:
                # return the manual turn command to null
                manual_turn_command = ''
                # make exite mode excutable again
                first_time_exit_round_detected = True
                second_time_exit_round_detected = True
                # return to normal mode
                driving_mode = 0
            # predict steering based on the CNN round recover model
            pred_steering = float(model_round_recover.predict(image))
            print("third part of round mode is active")





        # means a car stopped mode
        elif driving_mode == 9:
            # stop the car
            car.drive(0.0, 0.0)
            # get out of the main while loop
            break



        # means make cross walk pass
        elif driving_mode == 10:
            # using the go ahead model to predict
            # calculating the postion diffrence to be at least 1 unit
            diffrenc_postion = hypot(starting_postion[0] - car_postion[0], starting_postion[1] - car_postion[1])
            # use gps time to change the lane to right after 1 or more  simulation seconds
            current_time = int(gps.getGpsData()['timestamp'] * 1000) % 1000
            diffrenc_time = current_time - start_cross_walk_time
            # print('diffrenc_time', diffrenc_time)
            # print('diffrenc_postion', diffrenc_postion)
            if (diffrenc_time >= 1) and (diffrenc_postion >= 0.5):
                # return to lane keeping mode
                # return the manual turn command to null
                manual_turn_command = ''
                driving_mode = 0
            # predict the steering angles
            # reused the lane keeping for going ahead on intersection
            pred_steering = float(model_cross_walk_pass.predict(image))
            print("cross walk passing is active")



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
        if limit_frames:

            """					Predict steering          """
            # sending commands based on the predicting models
            # when the manual driving is off
            if not manual_driving:

                # if a recover from a close person stop
                # check last_excution_person_stop and time diffrence betwwen now and las excution time
                # if it was more than 2 seconds, increase the speed and return last_excution_person_stop to True
                if (not last_excution_person_stop) and (round(time.time(), 2) - last_excution_person_stop_time > 2):
                    car.drive(0.4, pred_steering)
                    last_excution_person_stop = True
                    lock = 'free'

                # if recovering from cross walk
                elif (not last_excution_cross_walk) and (round(time.time(), 2) - last_excution_cross_walk_time > 6):
                    # make sure that the lock is free
                    if lock == 'free':
                        car.drive(0.4, pred_steering)
                        last_excution_cross_walk = True

                # if recovering from tail
                elif (not last_excution_tail) and (round(time.time(), 2) - last_excution_tail_time > 1):
                    car.drive(0.4, pred_steering)
                    last_excution_tail = True

                # depending on the driving mode give predicted steering amgles with the current speed
                car.drive(get_current_speed(), pred_steering)
            print(get_current_speed(), get_current_steer())

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

                        # get the type of the object
                        object_detected = classes[class_id]
                        # then if it is close and on the right
                        if ((center_y + h / 2) > height / 3) and (center_x >= 2 * width / 3):
                            # print type and location
                            print("{} detected close on the right".format(classes[class_id]))
                            # taking actions:

                            # 1) first if its a cross_walk we slow down the speed to 0.2
                            if object_detected == 'cross_walk':
                                print("slowing down to speed 0.2")
                                # send commands to the car and send the same steering but
                                # different speed
                                last_excution_cross_walk = True
                                if lock == 'free':
                                    cross_walk_thread = threading.Thread(target=cross_walk_reaction, )
                                    cross_walk_thread.start()



                            # 2)  second if its a stop_sign we stop
                            elif object_detected == 'stop_sign':
                                # print("stop the car for 3 seconds")
                                # send commands to the car and send the same steering
                                # run the stop hanlde thread
                                stop_handle_thread = threading.Thread(target=stop_reaction, )
                                stop_handle_thread.start()
                                # join to prevent next instruction run with the new thread
                                stop_handle_thread.join()
                                last_excution_time = int(gps.getGpsData()['timestamp'] * 1000) % 1000




                            # 3) if its a high_way we increase speed
                            elif object_detected == 'high_way':
                                print("increasing the speed to 0.6")
                                # send commands to the car and send the same steering but
                                car.drive(0.6, get_current_steer())

                            # 4) fourth if its a high_way wnd  we stop
                            elif object_detected == 'end_high_way':
                                print("decreasing the speed to 0.4")
                                # send commands to the car and send the same steering but
                                car.drive(0.4, get_current_steer())

                            # 5) fifth if its a parking sign
                            elif object_detected == 'parking_sign':
                                if manual_turn_command == 'parking':
                                    # turn on parking mode if there is a spot
                                    parking_mode = True
                                    print("searching for an empty spot to park")

                            # 6) if we detect a spot and parking mode is on
                            elif object_detected == 'parking_spot_vertical' and parking_mode == True:
                                if ((center_y + h / 2) > 3 * height / 4) and (center_x >= 2 * width / 3):
                                    # turning parking procedures
                                    # send commands to the car and send the same steering
                                    print("Decreasing the speed down to 0.15")
                                    car.drive(0.1, get_current_steer())
                                    # save the latest yaw
                                    last_yaw = bno.getYaw()
                                    # switching prediction from the parking CNN model
                                    driving_mode = 7

                            # 7) if its a traffic light
                            elif object_detected == "traffic_light":
                                # make sure its close well
                                if ((center_y + h / 2) > 2 * height / 4) and (center_x >= 2 * width / 3):
                                    # determine which traffic light of the four based on the closest distance
                                    # test the traffic light module
                                    traffic_light_thread = threading.Thread(target=traffic_light_reaction,
                                                                            args=(car_postion,))
                                    traffic_light_thread.start()

                            # 8)  if its a round about sign
                            elif object_detected == 'rotating_road':
                                # save the satrting position
                                starting_postion = car_postion
                                # save the starting time
                                starting_rotate_time = int(gps.getGpsData()['timestamp'] * 1000) % 1000
                                # save the latest yaw
                                last_yaw = bno.getYaw()
                                # round mode activated
                                driving_mode = 8
                                print("round road is activated ")
                                # send commands to the car and send the same steering but
                                car.drive(0.4, get_current_steer())


                            # if its high way, the car cant wait until its so close to make a take over
                            # if the car on the basic right but close to the center, make a take over
                            elif object_detected == 'car':
                                if ((center_y + h / 2) > 3 * (height / 7)) \
                                        and (center_x <= 3 * (width / 4)) \
                                        and (allow_car_passing == True):
                                    print("car detected in highway, make a take over ")
                                    # make the driving mode 4, means take over
                                    # get Yaw before starting the turn
                                    if first_yaw_overtake == True:
                                        last_yaw = bno.getYaw()
                                    driving_mode = 4


                            #  if its exit rotate road
                            elif object_detected == 'rotate_road_exit':
                                # to make sure when to end road exit mode
                                # also dont activiate this mode if the current mode is not round road mode 8,11
                                if first_time_exit_round_detected and (driving_mode in [8, 11]):
                                    # if first turn chosed manually
                                    if manual_turn_command == 'first turn':
                                        print("exit round road activated ")
                                        # save the starting position
                                        starting_postion = car_postion
                                        # exit round mode activated
                                        driving_mode = 12
                                    # if second or thied turn chosed manually
                                    elif manual_turn_command in ['second turn', 'third turn']:
                                        # change first_time_exit_round_detected to false to not activate first
										# condition anymore
                                        first_time_exit_round_detected = False
                                        # save postion
                                        starting_postion_of_last_exit = car_postion


                                # if its not the first time and driving mode is round road 8,11 and
                                # second_time_exit_round_detected True
                                elif not first_time_exit_round_detected \
                                        and (driving_mode in [8, 11]) \
                                        and second_time_exit_round_detected:
                                    # calculate thr deffrence of the current postion and starting_postion_of_last_exit
                                    diffrenc_postion_exit_road = hypot(
                                        starting_postion_of_last_exit[0] - car_postion[0],
                                        starting_postion_of_last_exit[1] - car_postion[1])
                                    print('diffrenc_postion_exit_road', diffrenc_postion_exit_road)
                                    # make sure from the the distance deffrence
                                    if diffrenc_postion_exit_road >= 0.8:
                                        # if its the second turn
                                        if manual_turn_command == 'second turn':
                                            print("exit round road activated ")
                                            # save the starting position
                                            starting_postion = car_postion
                                            # exit round mode activated
                                            driving_mode = 12
                                        elif manual_turn_command == 'third turn':
                                            # change to third mode
                                            second_time_exit_round_detected = False
                                            # save postion
                                            starting_postion_of_last_exit = car_postion


                                # when its not the first neither second turn
                                elif not first_time_exit_round_detected \
                                        and (driving_mode in [8, 11]) \
                                        and not second_time_exit_round_detected:
                                    # calculate thr deffrence of the current postion and starting_postion_of_last_exit
                                    diffrenc_postion_exit_road = hypot(
                                        starting_postion_of_last_exit[0] - car_postion[0],
                                        starting_postion_of_last_exit[1] - car_postion[1])
                                    print('diffrenc_postion_exit_road', diffrenc_postion_exit_road)
                                    # make sure from the the distance deffrence
                                    if diffrenc_postion_exit_road >= 0.8:
                                        # if its the third turn
                                        if manual_turn_command == 'third turn':
                                            print("exit round road activated ")
                                            # save the starting position
                                            starting_postion = car_postion
                                            # exit round mode activated
                                            driving_mode = 12







                        # if the object close and on the left side
                        elif ((center_y + h / 2) > height / 3) and (center_x <= width / 3):
                            print("{} detected close on the left".format(classes[class_id]))
                            # person on left and very close
                            if object_detected == 'person':
                                if (center_y + h / 2) > (2 * height / 3):
                                    print("person ahead left, stop the car")
                                    # send commands to the car and send the same steering but
                                    # make a thread  to deal with pedistrian
                                    last_excution_person_stop = True
                                    # run a thread to deal this situation
                                    pedestrian_handle_thread = threading.Thread(target=pedestrian_reaction, )
                                    pedestrian_handle_thread.start()

                            # if its high way, the car cant wait until its so close to make a take over
                            # if the car on the basic left but close to the center, make a take over
                            elif object_detected == 'car':
                                if ((center_y + h / 2) > 3 * (height / 7)) \
                                        and (center_x > width / 4) \
                                        and (allow_car_passing == True):
                                    print("car detected in highway, make a take over ")
                                    # make the driving mode 4, means take over
                                    # get Yaw before starting the turn
                                    # true just for the first time
                                    if first_yaw_overtake == True:
                                        last_yaw = bno.getYaw()
                                    driving_mode = 4


                            #  if its exit rotate road
                            elif object_detected == 'rotate_road_exit':
                                # to make sure when to end road exit mode
                                # also dont activiate this mode if the current mode is not round road mode 8,11
                                if first_time_exit_round_detected and (driving_mode in [8, 11]):
                                    # if first turn chosed manually
                                    if manual_turn_command == 'first turn':
                                        print("exit round road activated ")
                                        # save the starting position
                                        starting_postion = car_postion
                                        # exit round mode activated
                                        driving_mode = 12
                                    # if second or thied turn chosed manually
                                    elif manual_turn_command in ['second turn', 'third turn']:
                                        # change first_time_exit_round_detected to false to not activate first condition anymore
                                        first_time_exit_round_detected = False
                                        # save postion
                                        starting_postion_of_last_exit = car_postion


                                # if its not the first time and driving mode is round road 8,11 and
                                # second_time_exit_round_detected True
                                elif not first_time_exit_round_detected \
                                        and (driving_mode in [8, 11]) \
                                        and second_time_exit_round_detected:
                                    # calculate thr deffrence of the current postion and starting_postion_of_last_exit
                                    diffrenc_postion_exit_road = hypot(
                                        starting_postion_of_last_exit[0] - car_postion[0],
                                        starting_postion_of_last_exit[1] - car_postion[1])
                                    print('diffrenc_postion_exit_road', diffrenc_postion_exit_road)
                                    # make sure from the the distance deffrence
                                    if diffrenc_postion_exit_road >= 0.8:
                                        # if its the second turn
                                        if manual_turn_command == 'second turn':
                                            print("exit round road activated ")
                                            # save the starting position
                                            starting_postion = car_postion
                                            # exit round mode activated
                                            driving_mode = 12
                                        elif manual_turn_command == 'third turn':
                                            # change to third mode
                                            second_time_exit_round_detected = False
                                            # save postion
                                            starting_postion_of_last_exit = car_postion


                                # when its not the first neither second turn
                                elif not first_time_exit_round_detected \
                                        and (driving_mode in [8, 11]) \
                                        and not second_time_exit_round_detected:
                                    # calculate thr deffrence of the current postion and starting_postion_of_last_exit
                                    diffrenc_postion_exit_road = hypot(
                                        starting_postion_of_last_exit[0] - car_postion[0],
                                        starting_postion_of_last_exit[1] - car_postion[1])
                                    print('diffrenc_postion_exit_road', diffrenc_postion_exit_road)
                                    # make sure from the the distance deffrence
                                    if diffrenc_postion_exit_road >= 0.8:
                                        # if its the third turn
                                        if manual_turn_command == 'third turn':
                                            print("exit round road activated ")
                                            # save the starting position
                                            starting_postion = car_postion
                                            # exit round mode activated
                                            driving_mode = 12





                        # if the object close and on the center
                        elif ((center_y + h / 2) > height / 3) and (
                                (center_x > width / 3) and (center_x < 2 * width / 3)):
                            # get the type of the object
                            object_detected = classes[class_id]
                            print("{} detected  on the center".format(classes[class_id]))
                            # taking actions:

                            # 1) first if its a person we stop the car
                            if object_detected == 'person':
                                if (center_y + h / 2) > (2 * height / 3):
                                    print("person ahead, stop the car")
                                    # send commands to the car and send the same steering but
                                    # make a thread  to deal with pedistrian
                                    last_excution_person_stop = True
                                    # run a thread to deal this situation
                                    pedestrian_handle_thread = threading.Thread(target=pedestrian_reaction, )
                                    pedestrian_handle_thread.start()





                            # 2) first if its a intersection we stop the car
                            elif object_detected == 'intersection':
                                # when the intersection is very close
                                if (center_y + h / 2) > (4 * height / 5):
                                    print("intersection is very close")

                                    # make side decision manual for now
                                    if manual_turn_command == 'right':
                                        print("make a right turn")
                                        # send commands to the car from the turn right model
                                        # get Yaw before starting the turn
                                        last_yaw = bno.getYaw()
                                        driving_mode = 1

                                    elif manual_turn_command == 'left':
                                        print("make a left turn")
                                        # send commands to the car from the turn left model
                                        # get Yaw before starting the turn
                                        last_yaw = bno.getYaw()
                                        driving_mode = 2

                                    elif manual_turn_command == 'go ahead':
                                        print("make a go ahead turn")
                                        # send commands to the car from the go ahead model
                                        # save the moment of starting go ahead  mode
                                        start_go_ahead_time = int(gps.getGpsData()['timestamp'] * 1000) % 1000
                                        # save the satrting position
                                        starting_postion = car_postion
                                        driving_mode = 3


                            # 3) if its a car
                            elif object_detected == 'car':
                                # make sure of the postion of the car when care over take is on
                                if ((center_y + h / 2) > 3 * (height / 7)) \
                                        and ((center_x > width / 3) and (center_x < 2 * width / 3)) \
                                        and (allow_car_passing == True):
                                    print("car detected in highway, make a take over ")
                                    # make the driving mode 4, means take over
                                    # get Yaw before starting the turn
                                    if first_yaw_overtake == True:
                                        last_yaw = bno.getYaw()
                                    driving_mode = 4

                                # 1) very close car and not a highway and not parking mode
                                elif ((center_y + h / 2) > (2 * height / 3) and (allow_car_passing == False)) and (
                                        driving_mode != 7):
                                    print("car ahead very close, stop the car and tail the car")
                                    # send commands to the car and send the same steering but
                                    # different speed
                                    last_excution_tail = True
                                    tail_stop_thread = threading.Thread(target=tail_stop_reaction)
                                    tail_stop_thread.start()


                            # 4) if it a cross walk
                            elif object_detected == 'cross_walk_road':
                                # if the cross walk road road very close
                                if (center_y + h / 2) > (2 * height / 3):
                                    print("cross walk passing is active")
                                    # save time before run the mode
                                    start_cross_walk_time = int(gps.getGpsData()['timestamp'] * 1000) % 1000
                                    # save the satrting position
                                    starting_postion = car_postion
                                    # cross walk road mode
                                    driving_mode = 10

                                # when the cross walk road is close
                                last_excution_cross_walk = True
                                if lock == 'free':
                                    cross_walk_thread = threading.Thread(target=cross_walk_reaction, )
                                    cross_walk_thread.start()



                            #  5) if its exit rotate road
                            elif object_detected == 'rotate_road_exit':
                                # to make sure when to end road exit mode
                                # also dont activiate this mode if the current mode is not round road mode 8,11
                                if first_time_exit_round_detected and (driving_mode in [8, 11]):
                                    # if first turn chosed manually
                                    if manual_turn_command == 'first turn':
                                        print("exit round road activated ")
                                        # save the starting position
                                        starting_postion = car_postion
                                        # exit round mode activated
                                        driving_mode = 12
                                    # if second or thied turn chosed manually
                                    elif manual_turn_command in ['second turn', 'third turn']:
                                        # change first_time_exit_round_detected to false to not activate first
										# condition anymore
                                        first_time_exit_round_detected = False
                                        # save postion
                                        starting_postion_of_last_exit = car_postion


                                # if its not the first time and driving mode is round road 8,11 and
                                # second_time_exit_round_detected True
                                elif not first_time_exit_round_detected \
                                        and (driving_mode in [8, 11]) \
                                        and second_time_exit_round_detected:
                                    # calculate thr deffrence of the current postion and starting_postion_of_last_exit
                                    diffrenc_postion_exit_road = hypot(
                                        starting_postion_of_last_exit[0] - car_postion[0],
                                        starting_postion_of_last_exit[1] - car_postion[1])
                                    print('diffrenc_postion_exit_road', diffrenc_postion_exit_road)
                                    # make sure from the the distance deffrence
                                    if diffrenc_postion_exit_road >= 0.8:
                                        # if its the second turn
                                        if manual_turn_command == 'second turn':
                                            print("exit round road activated ")
                                            # save the starting position
                                            starting_postion = car_postion
                                            # exit round mode activated
                                            driving_mode = 12
                                        elif manual_turn_command == 'third turn':
                                            # change to third mode
                                            second_time_exit_round_detected = False
                                            # save postion
                                            starting_postion_of_last_exit = car_postion


                                # when its not the first neither second turn
                                elif not first_time_exit_round_detected \
                                        and (driving_mode in [8, 11]) \
                                        and not second_time_exit_round_detected:
                                    # calculate thr deffrence of the current postion and starting_postion_of_last_exit
                                    diffrenc_postion_exit_road = hypot(
                                        starting_postion_of_last_exit[0] - car_postion[0],
                                        starting_postion_of_last_exit[1] - car_postion[1])
                                    print('diffrenc_postion_exit_road', diffrenc_postion_exit_road)
                                    # make sure from the the distance deffrence
                                    if diffrenc_postion_exit_road >= 0.8:
                                        # if its the third turn
                                        if manual_turn_command == 'third turn':
                                            print("exit round road activated ")
                                            # save the starting position
                                            starting_postion = car_postion
                                            # exit round mode activated
                                            driving_mode = 12

                        # making a list to hold all rectangles
                        boxes.append([x, y, w, h])
                        # making a list to hold all confidences
                        confidences.append(float(confidence))
                        # making a list to hold all classes
                        class_ids.append(class_id)

            """ drawing rectangles on objects after detecting """
            # to join multiple objects which in real the are only one
            indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
            # declare the font type to write on the images
            font = cv2.FONT_HERSHEY_PLAIN
            # some instructions to write on the image
            for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h = boxes[i]
                    label = str(classes[class_ids[i]])
                    if label == 'intersection':
                        continue
                    color = [0, 255, 0]
                    relative_pos = int(h * 1.3)
                    label = label + " " + str(round(confidences[i], 2))
                    cv2.rectangle(img, (x, y), (x + w, y + h), color, 1)
                    cv2.putText(img, label, (x, y + relative_pos), font, 1, color, 1)

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
        if key == ord('1'):
            manual_turn_command = 'first turn'
            print("first turn in the next rotate road")
        if key == ord('2'):
            manual_turn_command = 'second turn'
            print("second turn in the next rotate road")
        if key == ord('3'):
            manual_turn_command = 'third turn'
            print("third turn in the next rotate road")

    # when finished and the end of the thread
    print("car has been stopped \n END")
    # # turning the list to dictinary becasue its easy to deal with
    # all_data = { 'Im_paths': image_paths, 'steering': steering_list }
    # making a csv file to hold paths and steering DataSet
    with open('DataSet/driving_log.csv', 'ab') as f:
        writer = csv.writer(f)
        # titles
        writer.writerow(['Im_paths', 'steering'])
        # loop to save all recorded values
        for counter in range(len(image_paths)):
            writer.writerow([image_paths[counter], steering_list[counter]])
    # when finished and the end of the thread
    print("Dataset Has been saved. \n END")




"""                             THREE MAIN THREADS                                  """

# creating a thread to show and save images with the related steering
# angles on parallel with the control command
thread_img = threading.Thread(target=images_thread)
thread_img.start()




# creating a thread to do path plaining by maintaining a connection to the
# path planing TCP server
path_planing_thread = threading.Thread(target=path_planing)
path_planing_thread.start()




""" 									manual controling thread										"""


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
            car.drive(speed, get_current_steer())

    # if we press s
    if key.char == "s":
        if get_current_speed() > -max_speed:
            # decrease the speed by 0.2
            speed = get_current_speed() - speed_step
            # send commands to the car
            car.drive(speed, get_current_steer())

    # to steer right
    if key.char == "d":
        # when we reach the max steering which is 25
        # we will not increase the value of steering more
        # because even if we did, there will be no changes
        # on the real car
        if get_current_steer() < max_steering:
            steering = get_current_steer() + steering_step
            # send commands to the car
            car.drive(get_current_speed(), steering)

    # to steer left
    if key.char == "a":
        if get_current_steer() > -max_steering:
            steering = get_current_steer() - steering_step
            # send commands to the car
            car.drive(get_current_speed(), steering)

    # to show the steer and speed on the screen
    print(get_current_speed(), get_current_steer())


# maybe modified later
def on_release(key):
    pass


# Collect events until released
with Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()


