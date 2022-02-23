#!/usr/bin/env python3
import sys
import rospy
import cv2
import tf2_ros
import copy
import time

import re
import numpy as np
from math import *
from sensor_msgs.msg import Image, CameraInfo, LaserScan
# from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import PoseStamped
from visualization_msgs.msg import MarkerArray
from p_mrivadeneira_player.msg import detection_info
from image_geometry.cameramodels import PinholeCameraModel

from cv_bridge import CvBridge

global mins_target
global maxs_target
global mins_threat
global maxs_threat
global name

global image
global tf_buffer
global listener
global cam_info
global markers_copy
global area_target
global area_threat

global flag_goal_active
global flag_goal_achieved
global flag_target_close
global flag_threat_close

global goal_x
global goal_y
global goal_z
global target_x
global target_y
global target_z
global threat_x
global threat_y
global threat_z
global walls


def CameraInfoReceived(CI):

    global cam_info
    global image

    cam_info = CI

def GoalReceivedCallback(msg):

    global name
    global cam_info
    global flag_goal_active
    global goal_in_odom

    flag_goal_active = True

    # Transform goal to odom
    print('New Goal Received')
    target_frame = name + '/odom'
    goal_in_odom = tf_buffer.transform(msg, target_frame, rospy.Duration(1))

def MarkerArrayReceivedCallback(marker_array):

    global markers_copy
    global walls

    markers_copy = copy.deepcopy(marker_array)

    Z0A = False
    Z0B = False
    Z1A = False
    Z1B = False
    Z2A = False
    Z2B = False
    Z3A = False
    Z3B = False

    walls[0] = False
    walls[1] = False
    walls[2] = False
    walls[3] = False


    distance_limit = 1.5

    if len(marker_array.markers) > 0:
        # print(marker_array.markers[0].points[0])
        # print('\n\n\n\n')

        for i in range(0, len(marker_array.markers)):
            if len(marker_array.markers[i].points) <= 2:
                pass
            elif len(marker_array.markers[i].points) >= 15:
                # print('marker ' + str(marker_array.markers[i].id) + ' is a wall')

                for k in range(0, len(marker_array.markers[i].points)):

                    x = marker_array.markers[i].points[k].x
                    y = marker_array.markers[i].points[k].y

                    angle = atan2(x,-y)
                    distance = sqrt(x**2 + y**2)

                    if angle > -pi/6 and angle < 0 and distance < distance_limit:
                        Z0A = True
                    elif angle > 0 and angle < pi/6 and distance < distance_limit:
                        Z0B = True
                    elif angle > 2*pi/6 and angle < 3*pi/6 and distance < distance_limit:
                        Z1A = True
                    elif angle > 3*pi/6 and angle < 4*pi/6 and distance < distance_limit:
                        Z1B = True
                    elif angle > 5*pi/6 and distance < distance_limit:
                        Z2A = True
                    elif angle < -5*pi/6 and distance < distance_limit:
                        Z2B = True
                    elif angle > -4*pi/6 and angle < -3*pi/6 and distance < distance_limit:
                        Z3A = True
                    elif angle > -3*pi/6 and angle < -2*pi/6 and distance < distance_limit:
                        Z3B = True
            else:
                pass
                # print('marker ' + str(marker_array.markers[i].id) + ' is a body')

    if Z0A and Z0B:
        # print('There is a wall on the right')
        walls[0] = True
    if Z1A and Z1B:
        # print('There is a wall on the front')
        walls[1] = True
    if Z2A and Z2B:
        # print('There is a wall on the left')
        walls[2] = True
    if Z3A and Z3B:
        # print('There is a wall on the back')
        walls[3] = True

def Drawing(centroid_target, bounding_box_target, centroid_threat, bounding_box_threat):

    global image
    global cam_info
    global flag_goal_active
    global flag_goal_achieved
    global flag_target_close
    global flag_threat_close
    global goal_in_odom
    global starting_time
    global markers_copy
    global area_target
    global area_threat
    global goal_x
    global goal_y
    global goal_z
    global target_x
    global target_y
    global target_z
    global threat_x
    global threat_y
    global threat_z

    # -------------------------------------
    # Goal drawing
    # -------------------------------------

    if cam_info is None:
        pass
    else:
        cam = PinholeCameraModel()
        cam.fromCameraInfo(cam_info)

    color = (16, 184, 239)

    goal_x = 0
    goal_y = 0
    goal_z = 0

    if flag_goal_active == True:

        goal_copy = copy.deepcopy(goal_in_odom)
        goal_copy.header.stamp = rospy.Time.now()

        # Transform goal to base link
        goal_in_base_link = tf_buffer.transform(goal_copy, name + '/base_footprint', rospy.Duration(1))

        x_base_footprint = goal_in_base_link.pose.position.x
        y_base_footprint = goal_in_base_link.pose.position.y

        goal_x = x_base_footprint
        goal_y = y_base_footprint
        goal_z = goal_in_base_link.pose.position.z

        distance_to_goal = sqrt(x_base_footprint ** 2 + y_base_footprint ** 2)

        if distance_to_goal <= 0.2:
            print('Goal achieved!')

            starting_time = copy.deepcopy(time.perf_counter())
            flag_goal_achieved = True
            flag_goal_active = False

        else:

            # Transform goal to camera frame
            target_frame = name + '/camera_rgb_optical_frame'
            # print('The new target_frame is: ' + target_frame)

            goal = tf_buffer.transform(goal_copy, target_frame, rospy.Duration(1))

            x = goal.pose.position.x
            y = goal.pose.position.y
            z = goal.pose.position.z

            pix = cam.project3dToPixel((x, y, z))

            # print(pix)

            x_image = int(round(pix[0]))
            y_image = int(round(pix[1]))

            # print((x_base_footprint, y_base_footprint))
            # print(pix)
            if x_image < 1200 and x_image > 0 and y_image < 380 and y_image > 0 and x_base_footprint > 0.7:

                for i in range(0, 10):
                    plus = 5 * i
                    cv2.line(image, (x_image, y_image - plus), (x_image - 25, y_image - 10 - plus), color, 2)
                    cv2.line(image, (x_image, y_image - plus), (x_image + 25, y_image - 10 - plus), color, 2)

                cv2.putText(image, 'GOAL', (x_image - 40, y_image - 85), cv2.FONT_HERSHEY_PLAIN, 2, color, 2)
                cv2.putText(image, 'D = ' + str(round(distance_to_goal, 2)), (x_image - 35, y_image - 65),
                            cv2.FONT_HERSHEY_PLAIN, 1, color, 2)

            else:
                # print(pix)
                if x_base_footprint < 0.7:

                    x_image = x_image
                    if x_image > 1200:
                        x_image = 1200
                    if x_image < 0:
                        x_image = 0

                    y_image = 380

                    for i in range(0, 10):
                        plus = 5 * i
                        cv2.line(image, (x_image, y_image - plus), (x_image - 25, y_image - 10 - plus), color, 2)
                        cv2.line(image, (x_image, y_image - plus), (x_image + 25, y_image - 10 - plus), color, 2)

                    cv2.putText(image, 'GOAL', (x_image - 40, y_image - 85), cv2.FONT_HERSHEY_PLAIN, 2, color, 2)

                elif y_base_footprint < 0:

                    x_image = 1200
                    y_image = 250
                    for i in range(0, 10):
                        plus = 5 * i
                        cv2.line(image, (x_image - plus, y_image), (x_image - 10 - plus, y_image - 25), color, 2)
                        cv2.line(image, (x_image - plus, y_image), (x_image - 10 - plus, y_image + 25), color, 2)

                    cv2.putText(image, 'GOAL', (x_image - 150, y_image + 10), cv2.FONT_HERSHEY_PLAIN, 2, color, 2)

                else:

                    x_image = 0
                    y_image = 250
                    for i in range(0, 10):
                        plus = 5 * i
                        cv2.line(image, (x_image + plus, y_image), (x_image + 10 + plus, y_image - 25), color, 2)
                        cv2.line(image, (x_image + plus, y_image), (x_image + 10 + plus, y_image + 25), color, 2)

                    cv2.putText(image, 'GOAL', (x_image + 70, y_image + 10), cv2.FONT_HERSHEY_PLAIN, 2, color, 2)

    if flag_goal_achieved == True:

        finishing_time = time.perf_counter()

        if finishing_time - starting_time < 4:
            cv2.putText(image, 'GOAL ACHIEVED', (10, 50), cv2.FONT_HERSHEY_PLAIN, 2, color, 2)
        else:
            flag_goal_achieved = False

    # -------------------------------------
    # Target drawing
    # -------------------------------------
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

    # -------------------------------------
    # Target drawing
    # -------------------------------------

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

    # -------------------------------------
    # Markers drawing
    # -------------------------------------

    # Calculate transformation from base_footprint to camera frame
    target_frame = name + '/camera_rgb_optical_frame'

    marker_new = PoseStamped()
    marker_new.header.frame_id = name + '/base_footprint'
    marker_new.header.stamp = rospy.Time.now()
    marker_new.pose.orientation.w = 1

    dist_to_target_closest = 10000
    target_x = 0
    target_y = 0
    target_z = 0
    dist_to_threat_closest = 10000
    threat_x = 0
    threat_y = 0
    threat_z = 0

    if markers_copy is None:
        pass
    else:
        if len(markers_copy.markers) > 0:

            for i in range(0, len(markers_copy.markers)):
                for j in range(0, len(markers_copy.markers[i].points)):

                    marker_new.pose.position.x = markers_copy.markers[i].points[j].x
                    marker_new.pose.position.y = markers_copy.markers[i].points[j].y
                    marker_new.pose.position.z = markers_copy.markers[i].points[j].z

                    marker_transformed = tf_buffer.transform(marker_new, target_frame, rospy.Duration(1))  # Transform the markers the the camera's coordinates system

                    pix = cam.project3dToPixel((marker_transformed.pose.position.x, marker_transformed.pose.position.y, marker_transformed.pose.position.z))

                    x_image = int(round(pix[0]))
                    y_image = int(round(pix[1]))

                    if x_image < 1200 and x_image > 0 and y_image < 380 and y_image > 0 and marker_new.pose.position.x > 0:  # Just consider the pixel inside the camera space
                        image = cv2.circle(image, (x_image, y_image), 5, (239, 184, 16), -1)

                        # Closest to target
                        if centroid_target != (0, 0):
                            dist_to_target = sqrt((x_image - centroid_target[0])**2 + (y_image - centroid_target[1])**2)
                            if dist_to_target < dist_to_target_closest:
                                dist_to_target_closest = dist_to_target

                                target_x = marker_new.pose.position.x
                                target_y = marker_new.pose.position.y
                                target_z = marker_new.pose.position.z
                        else:
                            dist_to_target_closest = 10000
                        # Closest to target
                        if centroid_threat != (0, 0):
                            dist_to_threat = sqrt((x_image - centroid_threat[0]) ** 2 + (y_image - centroid_threat[1]) ** 2)
                            if dist_to_threat < dist_to_threat_closest:
                                dist_to_threat_closest = dist_to_threat

                                threat_x = marker_new.pose.position.x
                                threat_y = marker_new.pose.position.y
                                threat_z = marker_new.pose.position.z

                        else:
                            dist_to_threat_closest = 10000

        # print(dist_to_target_closest)
        # print(dist_to_threat_closest)
    if dist_to_target_closest <= 100 and area_target > 4500:
        flag_target_close = True
        cv2.putText(image, 'TARGET IS NEAR', (930, 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 200, 0), 2)
    else:
        flag_target_close = False

    if dist_to_threat_closest <= 100 and area_threat > 4500:
        flag_threat_close = True
        cv2.putText(image, 'THREAT IS NEAR', (930, 70), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 200), 2)
    else:
        flag_threat_close = False

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

def ImageReceivedCallback(image_message, pub_setpoint):

    global image
    global pixs
    global area_target
    global area_threat
    global flag_goal_active
    global flag_goal_achieved
    global flag_target_close
    global flag_threat_close
    global walls

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

    Drawing(centroid_target, bounding_box_target, centroid_threat, bounding_box_threat)

    if visualize == "true":
        # Show images
        # Drawing(centroid_target, bounding_box_target, centroid_threat, bounding_box_threat)
        # cv2.add(image, (-10, 100, -10, 0), dst=image, mask=Mask_target_closed)


        cv2.imshow('Camera ' + name, image)
        # cv2.imshow('Target mask', Mask_target_closed)
        # cv2.imshow('Threat mask', Mask_threat_closed)
        cv2.waitKey(20)

    # Publish data
    publish_msg = detection_info()
    publish_msg.target_center = centroid_target[0]
    publish_msg.target_area = area_target
    publish_msg.threat_center = centroid_threat[0]
    publish_msg.threat_area = area_threat
    publish_msg.flag_goal_achieved = flag_goal_achieved
    publish_msg.flag_goal_active = flag_goal_active
    publish_msg.goal.x = goal_x
    publish_msg.goal.y = goal_y
    publish_msg.goal.z = goal_z
    publish_msg.flag_target_close = flag_target_close
    if flag_target_close:
        publish_msg.target.x = target_x
        publish_msg.target.y = target_y
        publish_msg.target.z = target_z
    else:
        publish_msg.target.x = 0
        publish_msg.target.y = 0
        publish_msg.target.z = 0
    publish_msg.flag_threat_close = flag_threat_close
    if flag_threat_close:
        publish_msg.threat.x = threat_x
        publish_msg.threat.y = threat_y
        publish_msg.threat.z = threat_z
    else:
        publish_msg.threat.x = 0
        publish_msg.threat.y = 0
        publish_msg.threat.z = 0
    publish_msg.walls = walls

    pub_setpoint.publish(publish_msg)

def main():

    global cam_info
    global mins_target
    global maxs_target
    global mins_threat
    global maxs_threat
    global name
    global visualize
    global tf_buffer
    global listener
    global markers_copy
    global flag_goal_active
    global flag_goal_achieved
    global flag_target_close
    global flag_threat_close
    global starting_time
    global area_target
    global area_threat
    global goal_x
    global goal_y
    global goal_z
    global target_x
    global target_y
    global target_z
    global threat_x
    global threat_y
    global threat_z
    global walls

    # -------------------------------------
    # INITIALIZATION
    # -------------------------------------

    name = 'p_mrivadeneira_player'
    rospy.init_node(name, anonymous=False)

    # Teams recognition and configure of color limits for masks
    name = rospy.get_name()
    # name = name.strip('_td')
    # name_team_color = name.strip('/')
    # name_team_color = re.sub('[0-9]+', '', name_team_color)

    if name.find('blue') != -1:
        name = 'blue' + name[5]
        name_team_color = 'blue'
    elif name.find('green') != -1:
        name = 'green' + name[6]
        name_team_color = 'green'
    else:
        name = 'red' + name[4]
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

    pub_setpoint = rospy.Publisher('/detection', detection_info, queue_size=1)

    rospy.Subscriber('/camera/rgb/image_raw', Image, ImageReceivedCallback, pub_setpoint)
    rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, CameraInfoReceived)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, GoalReceivedCallback)
    rospy.Subscriber('/markers', MarkerArray, MarkerArrayReceivedCallback)


    # -------------------------------------
    # Definition of global variables
    # -------------------------------------
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    markers_copy = None

    cam_info = None

    flag_goal_achieved = False
    flag_goal_active = False
    flag_target_close = False
    flag_threat_close = False

    starting_time = time.perf_counter()

    area_target = 0
    area_threat = 0

    goal_x = 0
    goal_y = 0
    goal_z = 0
    target_x = 0
    target_y = 0
    target_z = 0
    threat_x = 0
    threat_y = 0
    threat_z = 0

    walls = [False, False, False, False]

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()