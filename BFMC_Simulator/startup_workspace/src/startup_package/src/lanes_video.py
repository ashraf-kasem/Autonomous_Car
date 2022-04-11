# importing the importatne laibriers
import cv2
import numpy as np
import matplotlib.pyplot as plt

# function to apply canny edge detector
def canny( image ):
    # transforing the image to gray scale
    gray = cv2.cvtColor( image, cv2.COLOR_RGB2GRAY)

    # applying the cany method to detect edges
    # parameters here define the min threshold and the Max threshold
    # for the canny detecter, you can read about canny detecter
    canny_output = cv2.Canny( gray ,50  , 150 )
    return canny_output

# a function to make a mask and apply it to the image
# to exctract the region of interest
def region_of_interset( image ):
    # hight is the number of rows in an image array
    hight = image.shape[0]
    # bulding a triangle consist of three piont that suround the area that we want
    triangle = np.array([
                            [(200, hight), (1100, hight ), (550, 250)]
                        ])
    # building a black image the same size of the image
    mask = np.zeros_like(image)
    # creating the mask by making the triangle area has white pixels
    cv2.fillPoly( mask, triangle, 255 )
    # apllying the mask to the image
    masked_image = cv2.bitwise_and( image, mask )
    return masked_image



# a function to drow lines on the output lane_image
def display_lines(image, lines):
    # building black image
    line_image = np.zeros_like(image)
    # loop in the cordinates of each lines
    # line format ( x1, y1, x2, y2  )
    if lines is not None:
        for line in lines :
            x1, y1, x2, y2 = line.reshape(4)
            # drow on the black image the lines
            # parameters ( the image that we want to drow on, the first line point
            #              the scond point, the color of the line, thickness of the line )
            cv2.line(line_image, (x1,y1), (x2,y2), (0,255,0), 10 )
    return line_image


# to make coordinates from the slope and the intercept (biase)
def make_coordinates( image, line_parameters ):
    slope, intercept = line_parameters
    # to get image height
    y1 = image.shape[0]
    # we want the line to start from the buttom to 3/5 from the image height
    y2 = int( y1 * ( 3/5 ))
    # simple equation to get x1 and x2 dependent on the y and slope and intercept (bias)
    x1 = int(( y1 - intercept ) / slope )
    x2 = int(( y2 - intercept ) / slope )
    # return line points
    return np.array( [ x1, y1, x2, y2 ])

def average_slope_intersect( image, lines ):
    # creating two lists that have all the slopes ans biases
    # for the right and left lines
    left_fit = []
    right_fit = []
    # looping in the lines line by line
    for line in lines :
        # get the cordinates for each line
        x1, y1, x2, y2 = line.reshape(4)
        # to get the parameters for each line (Xs of the line, Ys of the line, degree=1)
        parameters = np.polyfit( ( x1,x2 ), ( y1,y2 ), 1)
        # slope and intersept  ( biase )
        slope = parameters[0]
        intersept = parameters[1]
        # if the slope is less than zero thyen the line is in the left
        if slope < 0 :
            left_fit.append( ( slope,intersept ) )
        # if the slope is more than zero then the line is on the right
        else:
            right_fit.append( ( slope,intersept ) )

    # averging the slopes and biases for right and left lines
    left_fit_average = np.average( left_fit, axis=0 )
    right_fit_average = np.average( right_fit, axis=0 )
    # creating one left line and one right line using make_coordinates function
    left_line = make_coordinates(image,left_fit_average )
    right_line = make_coordinates(image,right_fit_average )
    # the return value is array of two optimazed lines
    return np.array([ left_line, right_line ])


cap = cv2.VideoCapture("test2.mp4")
while( cap.isOpened()):
    _ , frame = cap.read()
    # apllying canny edges detector
    canny_output = canny( frame )
    # apllying canny edges detector
    cropped_image = region_of_interset(canny_output)
    # creating lines from the cropped image
    # parameters ( the image, 2 pixles precsion, 1 degree precsion = pi/180
    #              thrsshold = 100 intersection, placeholder array ( empty array)
    #               min line length to be printed on output= 40 pixles
    #               max gap between lines to be conected = 5 pixles              )
    lines = cv2.HoughLinesP(    cropped_image, 2, np.pi/180, 100, np.array([]),
                                minLineLength = 40 ,maxLineGap = 5                  )

    averaged_lines = average_slope_intersect( frame, lines )

    # making the line image
    line_image = display_lines( frame, averaged_lines )

    # combine the orginal image with line line image
    # parameters : (first image, weight1, second image, weight2, gamma )
    combo_image = cv2.addWeighted(line_image, 0.8, frame, 1, 1 )
    cv2.imshow("result", combo_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows
