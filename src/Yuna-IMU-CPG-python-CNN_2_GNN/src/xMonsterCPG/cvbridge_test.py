#!/usr/bin/env python2
from __future__ import print_function

import roslib

roslib.load_manifest('xMonsterCPG')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time

def callback(data):
    global bridge
    # global img_num


    try:
        #rate = rospy.Rate(25.0)
        dir_pub = rospy.Publisher('further_direction', String, queue_size = 1)
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")


        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lowerb_hsv = np.array([0, 43, 46])  # range of red color in hsv
        upperb_hsv = np.array([10, 255, 255])
        lowerg_hsv = np.array([35, 43, 46])  # range of green color in hsv
        upperg_hsv = np.array([77, 255, 255])
        mask1 = cv2.inRange(hsv, lowerb_hsv, upperb_hsv)  # build mask
        mask2 = cv2.inRange(hsv, lowerg_hsv, upperg_hsv)
        # use findcounter to get the object outline
        # print(cv2.__version__)
        r, cnts, h = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE, )

        # do the next step if the contour more than 0
        if len(cnts) > 0:
            # find the contour which area is biggest
            c = max(cnts, key=cv2.contourArea)
            # use the minimum circle out the object
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            # compute the moment of the object
            M = cv2.moments(c)
            # compute the center of the gravity
            if int(M["m00"]) > 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                direction = (int(M["m10"] / M["m00"]) - 320, int(M["m01"] / M["m00"]) - 240)

                # cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # cv2.circle(cv_image, center, 5, (0, 0, 255), -1)

                

                if abs(direction[0]) <= 120:
                    # print('go straight')
                    next_direction = 0
                    object_dir = 0

                elif direction[0] > 120:
                    # print('turn right')
                    next_direction = 1
                    object_dir = 1
                elif direction[0] < -120:
                    # print('turn left')
                    next_direction = -1
                    object_dir = 2
                # print(width)
                else:
                    # print('turn around')
                    next_direction = 2
                    object_dir = 3

        else:
            # print('no object')
            next_direction = 2
            object_dir = 3

        cv2.imshow("Image window", cv_image)
        # print(img_num)
        # if object_dir==3:
        #     img_num = img_num + 1
        #     image_name = "./data/noobject/"+str(img_num) + '_' + str(object_dir) + ".jpg"
        #     cv2.imwrite(image_name, cv_image)

        cv2.waitKey(40)

        dir_pub.publish(str(next_direction))
        #rate.sleep()
        #rospy.loginfo(str(next_direction))
        

        #time.sleep(0.1)
    except CvBridgeError as e:
        print(e)

    # try:
    # self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    # print(e)


def sendinfo():
    # global img_num
    img_num = 0
    while not rospy.is_shutdown():
        #import torch
        rospy.init_node('image_processor', anonymous=True)
        global bridge

        bridge = CvBridge()

        rospy.Subscriber("/right_r200/camera/color/image_raw", Image, callback, queue_size=1)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    sendinfo()

