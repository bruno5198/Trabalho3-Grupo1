#!/usr/bin/env python3
import sys
import rospy
import cv2
import re
import numpy as np
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from p_mrivadeneira_player.msg import camera_detection


from cv_bridge import CvBridge

global mins_target
global maxs_target
global mins_threat
global maxs_threat
global name

global markers

markers = []

def drawing(image,  centroid_target, bounding_box_target, centroid_threat, bounding_box_threat):

    if centroid_target != (0, 0):

        # Drawing a cross
        x = centroid_target[0]
        y = centroid_target[1]
        w = 3
        h = 0
        cv2.rectangle(image, (x - w, y - h), (x + w, y + h), (0, 200, 0), 2)
        cv2.rectangle(image, (x - h, y - w), (x + h, y + w), (0, 200, 0), 2)

        # Drawing a Bounding_Box
        x1 = bounding_box_target[0]
        y1 = bounding_box_target[1]
        x2 = x1 + bounding_box_target[2]
        y2 = y1 + bounding_box_target[3]
        cv2.rectangle(image, (x1, y1), (x2, y2), (50, 175, 50), 2)

        # Text
        cv2.putText(image, 'TARGET', (x - 60, y1 - 10), cv2.FONT_HERSHEY_PLAIN, 2, (0, 200, 0), 2)

    else:
        image = image

    if centroid_threat != (0, 0):

        # Drawing a cross
        x = centroid_threat[0]
        y = centroid_threat[1]
        w = 3
        h = 0
        cv2.rectangle(image, (x - w, y - h), (x + w, y + h), (0, 0, 200), 2)
        cv2.rectangle(image, (x - h, y - w), (x + h, y + w), (0, 0, 200), 2)

        # Drawing a Bounding_Box
        x1 = bounding_box_threat[0]
        y1 = bounding_box_threat[1]
        x2 = x1 + bounding_box_threat[2]
        y2 = y1 + bounding_box_threat[3]
        cv2.rectangle(image, (x1, y1), (x2, y2), (50, 50, 175), 2)

        # Text
        cv2.putText(image, 'THREAT', (x - 60, y1 - 10), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 200), 2)

    else:
        image = image

    return image

def DetectObject(Mask):

    try:
        cnts, _ = cv2.findContours(Mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cnt = max(cnts, key=cv2.contourArea)
        Mask_target_unique = np.zeros(Mask.shape, np.uint8)
        cv2.drawContours(Mask_target_unique, [cnt], -1, 255, cv2.FILLED)
        Mask_target_unique = cv2.bitwise_and(Mask, Mask_target_unique)
        # If there is a black mask, there are not objects
    except:
        Mask_target_unique = Mask

    # Morphologicall Transformation - Closing
    kernel = np.ones((5, 5), np.uint8)
    Mask_target_closed = cv2.dilate(Mask_target_unique, kernel, iterations=20)
    Mask_target_closed = cv2.erode(Mask_target_closed, kernel, iterations=20)

    # Find centroid
    try:
        M = cv2.moments(Mask_target_closed)
        Cx = int(M["m10"] / M["m00"])
        Cy = int(M["m01"] / M["m00"])
    except:
        Cx = 0
        Cy = 0

    centroid = (Cx, Cy)

    # print('The centroid of the object is in = ' + str(centroid))

    # Find Bounding Box
    try:
        x, y, w, h = cv2.boundingRect(cnt)
    except:
        x = 0
        y = 0
        w = 0
        h = 0
    Bounding_Box = (x, y, w, h)

    return Mask_target_closed, centroid, Bounding_Box

def LaserScanPointsReceived(marker_array):

    pass
    # print(marker_array)
    # img = np.zeros((400, 1200))
    #
    # if len(marker_array.markers) > 0:
    #     # print(marker_array.markers[0].points[0])
    #     # print('\n\n\n\n')
    #
    #     for i in range(0, len(marker_array.markers)):
    #         if len(marker_array.markers[i].points) <= 2:
    #             pass
    #         elif len(marker_array.markers[i].points) >= 15:
    #             print('marker ' + str(marker_array.markers[i].id) + ' is a wall')
    #         else:
    #             print('marker ' + str(marker_array.markers[i].id) + ' is a body')
    #             # marker = (marker_array.markers[i].points[0].x, marker_array.markers[i].points[0].y, marker_array.markers[i].points[0].z)
    #             #
    #             # markers.append(marker)
    #             print(marker_array.markers[i].points)
    #             img[marker_array.markers[i].points[0].z+200][marker_array.markers[i].points[0].y+600] = 255

    # cv2.imshow('Markers', img)
    # cv2.waitKey(20)

def PixelsReceived(pix):


    img = np.zeros((400, 1200))
    pixs = pix
    pixs.x = int(pixs.x)
    pixs.y = int(pixs.y)
    pixs.z = int(pixs.z)

    print(pixs)
    img = cv2.circle(img, (pixs.x, pixs.y), 20, (255,255,255), -1)

    cv2.imshow('PIX',img)
    cv2.waitKey(20)

    print(img[pixs.y][pixs.x])




def ImageReceivedCallback(image_message, pub_setpoint):

    global pixs

    # rospy.loginfo("The image from the camera was received")

    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_message, desired_encoding='bgr8')        # Convert camera image to cv2 image

    # -------------------------------------
    # Target detection
    # -------------------------------------

    Mask_target = cv2.inRange(image, mins_target, maxs_target)
    Mask_target_closed, centroid_target, bounding_box_target = DetectObject(Mask_target)

    area_target = np.count_nonzero(Mask_target_closed == 255)
    # print('The area of the target is = ' + str(area_target))

    # -------------------------------------
    # Threat detection
    # -------------------------------------

    Mask_threat = cv2.inRange(image, mins_threat, maxs_threat)
    Mask_threat_closed, centroid_threat, bounding_box_threat = DetectObject(Mask_threat)

    area_threat = np.count_nonzero(Mask_threat_closed == 255)
    # print('The area of the threat is = ' + str(area_threat))

    # Draw circles
    # if len(pixs) > 0:
    # cv2.circle(image, (pixs[1], pixs[0]), 5, (255,0,0), -1)

    if visualize == "true":
        # Show images
        image = drawing(image, centroid_target, bounding_box_target, centroid_threat, bounding_box_threat)
        # cv2.add(image, (-10, 100, -10, 0), dst=image, mask=Mask_target_closed)  # Recognize mask in webcam window


        # cv2.imshow('Camera' + name, image)
        # cv2.imshow('Target mask', Mask_target_closed)
        # cv2.imshow('Threat mask', Mask_threat_closed)
        # cv2.waitKey(20)

    # Publish data
    publish_array = camera_detection()
    publish_array.target_center = centroid_target[0]
    publish_array.target_area = area_target
    publish_array.threat_center = centroid_threat[0]
    publish_array.threat_area = area_threat

    pub_setpoint.publish(publish_array)

def main():

    global mins_target
    global maxs_target
    global mins_threat
    global maxs_threat
    global name
    global visualize

    # -------------------------------------
    # INITIALIZATION
    # -------------------------------------

    rospy.init_node('p_mrivadeneira_td', anonymous=False)

    # Teams recognition and configure of color limits for masks
    name = rospy.get_name()
    # name = name.strip('_td')
    # name_team_color = name.strip('/')
    # name_team_color = re.sub('[0-9]+', '', name_team_color)

    if name.find('blue') != -1:
        name_team_color = 'blue'
    elif name.find('green') != -1:
        name_team_color = 'green'
    else:
        name_team_color = 'red'

    range_of_limits_red = {"limits": {"B": {"min": 0, "max": 50}, "G": {"min": 0, "max": 50}, "R": {"min": 100, "max": 255}}}
    range_of_limits_green = {"limits": {"B": {"min": 0, "max": 50}, "G": {"min": 100, "max": 255}, "R": {"min": 0, "max": 50}}}
    range_of_limits_blue = {"limits": {"B": {"min": 100, "max": 255}, "G": {"min": 0, "max": 50}, "R": {"min": 0, "max": 50}}}

    if name_team_color == 'blue':
        range_of_limits_target = range_of_limits_red
        range_of_limits_threat = range_of_limits_green
    elif name_team_color == 'green':
        range_of_limits_target = range_of_limits_blue
        range_of_limits_threat = range_of_limits_red
    else:
        range_of_limits_target = range_of_limits_green
        range_of_limits_threat = range_of_limits_blue

    mins_target = np.array([range_of_limits_target['limits']['B']['min'], range_of_limits_target['limits']['G']['min'], range_of_limits_target['limits']['R']['min']])
    maxs_target = np.array([range_of_limits_target['limits']['B']['max'], range_of_limits_target['limits']['G']['max'], range_of_limits_target['limits']['R']['max']])

    mins_threat = np.array([range_of_limits_threat['limits']['B']['min'], range_of_limits_threat['limits']['G']['min'], range_of_limits_threat['limits']['R']['min']])
    maxs_threat = np.array([range_of_limits_threat['limits']['B']['max'], range_of_limits_threat['limits']['G']['max'], range_of_limits_threat['limits']['R']['max']])


    args = rospy.myargv(argv=sys.argv)
    visualize = args[1]

    # -------------------------------------
    # Definition Publisher & Subscriber
    # -------------------------------------

    pub_setpoint = rospy.Publisher('/setpoint_msg/coords', camera_detection, queue_size=1)

    rospy.Subscriber('/camera/rgb/image_raw', Image, ImageReceivedCallback, pub_setpoint)
    rospy.Subscriber('/markers', MarkerArray, LaserScanPointsReceived)
    rospy.Subscriber('/blue1/pixels', Point, PixelsReceived)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()