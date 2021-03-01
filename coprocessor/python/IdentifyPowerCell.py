#Imports
import cv2
import numpy as np
import math

#Shows a image with flipped colors
cam = cv2.VideoCapture(0)
while True:
    ret = False
    while not ret:
        ret, frame = cam.read()
    #print(frame.shape, frame[:,:,::-1].shape)

    #Generates a Box
    subArray = frame[200:280, 280:360, :].copy()

    #Makes subArray into an hsv image
    hsvImage = cv2.cvtColor(subArray.copy(), cv2.COLOR_BGR2HSV)

    #Draws A Box
    topLeftCorner = (280, 200)
    bottomRightCorner = (360, 280)
    colorOfTheBox = (0, 0, 255)
    cv2.rectangle(frame, topLeftCorner, bottomRightCorner, colorOfTheBox, 3)

    #Shows a regular image with the box
    fixedImage = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2RGB)

    #Creates a copy of hsvImage that is divided to fit in the parameters of cv2.cvtColor
    hsvFloat = hsvImage.copy() / 180

    #Finding theta for transforming to radians

    #Creates an image that is 640 by 480 by 7
    mathImage = np.zeros((80, 80, 7))
    mathImage[:, :, 0] = hsvImage[:, :, 0].copy() * (math.pi / (180 / 2) )

    #Finding x and y orbits for transforing to radians
    #Finding x
    mathImage[:, :, 1] = np.cos(mathImage[:, :, 0])
    x = mathImage[:, :, 1].mean()

    #Finding y
    mathImage[:, :, 2] = np.sin(mathImage[:, :, 0])
    y = mathImage[:, :, 2].mean()

    fixedSmallImage = cv2.cvtColor(subArray.copy(), cv2.COLOR_BGR2RGB)

    yellow = (0.4959, 0.8670)

    def sqr(value):
        return value * value

    x1, y1 = yellow
    distanceFromYellow = math.sqrt(sqr(x1 - x) + sqr(y1 - y))

    error = 0.7

    if distanceFromYellow < error:
        print("ball")
    else:
        print("probably no ball")
cam.release();