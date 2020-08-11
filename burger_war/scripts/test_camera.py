
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import sendIdToJudge
from tf.transformations import euler_from_quaternion
import subprocess
import matplotlib.pyplot as plt

dir = "/home/majima/catkin_ws/src/burger_war/burger_war/scripts/first_position/"

def area(img):
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    hsvLower = np.array([0, 128, 0])
    hsvUpper = np.array([30, 255, 255])
    mask1 = cv2.inRange(hsv, hsvLower, hsvUpper)

    hsvLower = np.array([150, 128, 0])
    hsvUpper = np.array([179, 255, 255])
    mask2 = cv2.inRange(hsv, hsvLower, hsvUpper)
    
    mask = mask1 + mask2

    masked_hsv = cv2.bitwise_and(img, img, mask=mask)
    gray = cv2.cvtColor(masked_hsv,cv2.COLOR_BGR2GRAY)

    cv2.imwrite(dir + 'gray_cvtcolr.jpg', gray)

    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
    # Find circles with HoughCircles
    circles = cv2.HoughCircles(thresh, cv2.HOUGH_GRADIENT, 1, minDist=200, param1=200, param2=10, minRadius=0)
    #circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, minDist=1, param1=300, param2=5, minRadius=0)
    # Draw circles
    if circles is not None:
        print("x,y,r:",circles[0])
        circles = np.round(circles[0, :]).astype("int")
        cv2.circle(img,(circles[0][0],circles[0][1]),circles[0][2], (36,255,12), 3)
        #cv2.circle(thresh,(circles[0][0],circles[0][1]),circles[0][2], (36,255,12), 3)
    
        for (x,y,r) in circles:
            cv2.circle(img, (x,y), r, (36,255,12), 3)
            #print(x,y,r)
    
    cv2.imwrite(dir + 'thresh.png', thresh)
    cv2.imwrite(dir + 'circle.png', img)
    cv2.waitKey()

    ret,thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY)
    image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    return cv2.contourArea(contours[0])

im = cv2.imread(dir + 'first_position_0.5.png')
area(im)
#print(S)