#Imports
import cv2
import matplotlib 
from matplotlib import pyplot as plt
import numpy as np
import math
from networktables import NetworkTables as nt

#Shows a image with flipped colors
cam = cv2.VideoCapture(0)

#Initializing the Network Tables
nt.initialize(server = "10.3.42.2")
table = nt.getTable("SmartDashboard")

#Main loop
while True:
    ret = False
    while not ret:
        ret, frame = cam.read()

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
     #print("\nDouble the numbers when checking for correctness.\n")
     #print(hsvImage[:9, :9, 0])

    #Finding theta for transforming to radians

    #Creates an image that is 640 by 480 by 7
    mathImage = np.zeros((80, 80, 7))
     #print(mathImage.shape)
    mathImage[:, :, 0] = hsvImage[:, :, 0].copy() * (math.pi / (180 / 2) )

    #Finding x and y orbits for transforing to radians
    #Finding x
    mathImage[:, :, 1] = np.cos(mathImage[:, :, 0])
    x = mathImage[:, :, 1].mean()

    #Finding y
    mathImage[:, :, 2] = np.sin(mathImage[:, :, 0])
    y = mathImage[:, :, 2].mean()

     #print("\nTheta")
     #print(mathImage[:9, :9, 0])
     #print("\nX")
     #print(x)
     #print("\nY")
     #print(y)

    #Prints the result and fixedImage
     #print(subArray[3, 3, :])

    #Creates a fixedSmallImage
    fixedSmallImage = cv2.cvtColor(subArray.copy(), cv2.COLOR_BGR2RGB)

    #A method that squares a number
    def sqr(value):
        return value * value

    #Creates a dictionary that represents the color's values
    colors = {"yellow" : (0.4959, 0.8670),"red" : (0.9940, -0.1054), "green" : (-0.9977, 0.010), "blue" : (-0.8497, -0.5268)}

    #Finds the distances between the picture's value and the colors' values
    distances = []
    for i in colors:
        x1, y1 = colors[i]
        distanceFromColors = math.sqrt(sqr(x1 - x) + sqr(y1 - y))
         #print(distanceFromColors)
        distances.append(distanceFromColors)

         #print(distances)

    #Finds the color
    least = 2
    for i in distances:
        if i < least:
            least = i      
    print(list(colors)[distances.index(least)])
    
    #Puts the value in a Network Table
    table.putString("color", list(colors)[distances.index(least)])

#Stops Camera Feed
cam.release()