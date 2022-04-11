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
import numpy as np

from keras.models import load_model

import threading
import rospy
import cv2
from time import sleep
from pynput.keyboard import Key, Listener
from datetime import datetime
from PIL import Image
import os
import csv
import h5py

# This line should be the first line in your program
rospy.init_node('main_node', anonymous=True)

# build object from the car handler
cam = CameraHandler()
print("Camera loaded")

car = Controller()
print("Controller loaded")





# load the model
model = load_model('model_v6.h5')


# to preprocess the image before making a prediction
def img_preprocess(img):
    img = img[100:450,:,:]
    img = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
    img = cv2.GaussianBlur(img,  (3, 3), 0)
    img = cv2.resize(img, (200, 66))
    img = img/255
    return img


steering = 0.0
speed = 0.0
# counter to limit recording to 10 frame per second
limit_frames = 0



# to loop in frames
while True:

    # show the image on the previw
    cv2.imshow("Frame preview", cam.getImage())
    key = cv2.waitKey(1)

    # preprocess the current frame
    image = img_preprocess(cam.getImage())
    image = np.array([image])

    # to record 10 fps
    if limit_frames % 10 == 0:
        # using the model to predict
        steering = float(model.predict(image))
        # send commands to the car
        car.drive(1, steering )
        print('speed {0}, steering {1}'.format(speed, steering))

    # to not record thousends of images, and reduce the data Set
    # the loop will preduce 100 frames per seconds
    limit_frames += 1
    sleep(0.01)


    # to exit
    if key == ord('q'):
        cv2.destroyAllWindows()
        print("car stopped")
        break
