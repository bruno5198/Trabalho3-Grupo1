#!/usr/bin/env python3
import copy
import math
import random
import time
from tkinter import SUNKEN

import rospy
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped, Point
import image_view
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
import tf2_geometry_msgs
import sys
import tkinter as tk

publisher = rospy.Publisher('/markers', MarkerArray, queue_size=1)

class Driver():

    # my_team = None
    # hunter_team = None
    # prey_team = None

    def __init__(self):
        # # Goal creation.
        self.goal = PoseStamped()
        self.manual_goal_active = False
        self.closer_body_dist = 0
        self.back_image_to_show = None
        self.image_to_show = None
        self.robot_general_state = ''
        self.robot_state = ''
        self.robot_hunter_state = ''
        self.robot_prey_state = ''
        self.name = ''
        self.hunter_size = 0
        self.prey_size = 0
        self.back_hunterPosition = 0
        self.closer_body = False
        # self.dist_to_body = 0
        self.regions = {}
        self.avoid_wall = False
        self.my_team = None
        self.prey_detected = False
        self.last_prey_detected = 'Right'
        self.hunter_detected = False
        self.back_hunter_detected = False

        # Linear anda angular speed definition.
        self.angle = 0
        self.speed = 0

        # Initialization of variables to save teams constitution.
        self.green_team = None
        self.blue_team = None
        self.red_team = None

        self.counter = 0
        self.wall_detected = False

        self.initialConfig()

        # Publisher creation.
        self.publisher_command = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.goal_subscriber_camera = rospy.Subscriber(self.name + '/camera/rgb/image_raw', Image, self.handleImageCallback)

        self.goal_subscriber_back_camera = rospy.Subscriber(self.name + '/back_camera/rgb/image_raw', Image, self.handleBackImageCallback)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.sendCommandCallback)

        self.laser_scan = rospy.Subscriber("/" + self.name + "/scan", LaserScan, self.laserScanMessageReceivedCallback)

        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goalReceivedCallback)


    def initialConfig(self):
        # -------------------------------- Get team player names -------------------------------
        self.red_team = rospy.get_param('/red_players')
        self.blue_team = rospy.get_param('/blue_players')
        self.green_team = rospy.get_param('/green_players')
        # --------------------------------------------------------------------------------------

        # ----------------------------------- Get player name ----------------------------------
        self.name = rospy.get_name()
        self.name = self.name.strip('/')                    # Remove initial '/'.
        # --------------------------------------------------------------------------------------

        # -------------------------------- Initialize variables --------------------------------
        hunter_team = []
        prey_team = []
        # --------------------------------------------------------------------------------------

        # ------------ Set player's team, prey's and hunter's according to his color -----------
        if self.name in self.red_team:
            self.my_team = self.red_team
            hunter_team = self.blue_team
            prey_team = self.green_team
        elif self.name in self.blue_team:
            self.my_team = self.blue_team
            hunter_team = self.green_team
            prey_team = self.red_team
        elif self.name in self.green_team:
            self.my_team = self.green_team
            hunter_team = self.red_team
            prey_team = self.blue_team
        else:
            rospy.logfatal('Invalid player name!')
        # --------------------------------------------------------------------------------------

        # ---------------- Print player's team, prey's and hunter's information ----------------
        self.robot_state = 'My name is ' + str(self.name) + ". I am team " + str(self.my_team) + ". I am hunting " + str(hunter_team) + " and feeling from " + str(prey_team)
        rospy.loginfo('My name is ' + str(self.name) + ". I am team " + str(self.my_team) + ". I am hunting " + str(hunter_team) + " and feeling from " + str(prey_team))
        # --------------------------------------------------------------------------------------


    def goalReceivedCallback(self, msg):
        # Verify if goal set manually is on odom frame.
        self.goal = copy.copy(msg)  # Store goal.
        self.manual_goal_active = True


    def handleImageCallback(self, msg):
        mass_center_prey, mass_center_hunter, cv_image, prey_mask, hunter_mask = self.imageTreatment(msg, 'Front')

        # ----------------------------------- Prey detection -----------------------------------
        self.robot_prey_state = 'Prey undetected!'
        if mass_center_prey is None:
            self.prey_detected = False
            rospy.loginfo('Prey undetected!')
        else:
            self.preyPosition = mass_center_prey[0] - (cv_image.shape[1] / 2)
            self.prey_size = np.sum(prey_mask == 255)
            if self.prey_size != 0:
                if self.preyPosition > 0:
                    rospy.loginfo('Prey detected at right side!')
                    self.last_prey_detected = 'Right'
                elif self.preyPosition < 0:
                    rospy.loginfo('Prey detected at left side!')
                    self.last_prey_detected = 'Left'
                self.prey_detected = True
                self.robot_prey_state = 'Prey detected!'
            else:
                rospy.loginfo('Prey undetected!')
                self.prey_detected = False
        # --------------------------------------------------------------------------------------

        # ---------------------------------- Hunter detection ----------------------------------
        self.robot_hunter_state = 'Hunter undetected!'
        if mass_center_hunter is None:
            self.hunter_detected = False
            rospy.loginfo('Hunter undetected!')
        else:
            self.hunterPosition = mass_center_hunter[0] - (cv_image.shape[1] / 2)
            self.hunter_size = np.sum(hunter_mask == 255)
            if self.hunter_size != 0:
                if (self.closer_body_dist > 3.0) or (self.closer_body_dist == 0):
                    self.robot_hunter_state = 'Hunter detected far away!'
                else:
                    self.robot_hunter_state = 'Hunter detected!'
                rospy.loginfo('Hunter detected!')
                self.hunter_detected = True
            else:
                rospy.loginfo('Hunter undetected!')
                self.hunter_detected = False
        # --------------------------------------------------------------------------------------

    def handleBackImageCallback(self, msg):
        back_mass_center_prey, back_mass_center_hunter, cv_back_image, back_prey_mask, back_hunter_mask = self.imageTreatment(msg, 'Back')

        # ---------------------------------- Hunter detection ----------------------------------
        if back_mass_center_hunter is None:
            self.back_hunter_detected = False
            rospy.loginfo('Hunter undetected behind robot!')
        else:
            self.back_hunterPosition = back_mass_center_hunter[0] - (cv_back_image.shape[1] / 2)
            if np.sum(back_hunter_mask == 255) != 0:
                if (self.closer_body_dist > 3.0) or (self.closer_body_dist == 0):
                    self.robot_hunter_state = 'Hunter detected behind robot far away!'
                elif self.closer_body_dist <= 3.0:
                    self.robot_hunter_state = 'Hunter detected behind robot!'
                    rospy.loginfo('Hunter detected behind robot!')
                    self.back_hunter_detected = True
            else:
                self.back_hunter_detected = False
        # --------------------------------------------------------------------------------------

        # --------------------------- Show front and back camera image -------------------------
        horizontalAppendedImg = np.hstack((self.image_to_show, self.back_image_to_show))       # Concatenate front and back camera image at one window.

        # ................ Add text related to robot state to image .................
        text = self.robot_state + ' | ' + self.robot_prey_state + ' | ' + self.robot_hunter_state + ' | General state of the robot: ' + self.robot_general_state
        font = cv2.FONT_HERSHEY_SIMPLEX
        coordinates = (5, 15)
        fontScale = 0.45
        fontColor = (0, 0, 0)
        thickness = 1
        lineType = cv2.LINE_AA
        # y0, dy = 15, 20

        horizontalAppendedImg = cv2.rectangle(horizontalAppendedImg, (0, 0), (2000, 25), (255, 255, 255), -1)   # Draw white background rectangle.

        # for i, line in enumerate(text.split('\n')):
        #     y = y0 + i * dy
        #     horizontalAppendedImg = cv2.putText(horizontalAppendedImg, line, (5, y), font, fontScale, fontColor, thickness, lineType)
        horizontalAppendedImg = cv2.putText(horizontalAppendedImg, text, coordinates, font, fontScale, fontColor, thickness, lineType)
        # ...........................................................................

        cv2.imshow('Player ' + str(self.name) + ' (Front and Back camera image)', horizontalAppendedImg)
        cv2.waitKey(3)
        # --------------------------------------------------------------------------------------

    def drawingFunction(self, cv_image, mask, drawing_color):
        # ------------------------ Visual identification of prey/hunter ------------------------
        image_text = ['Prey', 'Hunter']
        for ii in range(2):
            contours, _ = cv2.findContours(mask[ii], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            contours_poly = [None] * len(contours)
            boundRect = [None] * len(contours)
            centers = [None] * len(contours)
            radius = [None] * len(contours)
            for i, c in enumerate(contours):
                contours_poly[i] = cv2.approxPolyDP(c, 3, True)
                boundRect[i] = cv2.boundingRect(contours_poly[i])
                centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])

            color = drawing_color[ii]
            thickness = 2
            if len(contours) == 1:
                cv2.rectangle(cv_image, (int(boundRect[0][0]), int(boundRect[0][1])), (int(boundRect[0][0] + boundRect[0][2]), int(boundRect[0][1] + boundRect[0][3])), color, thickness)
                cv2.putText(cv_image, image_text[ii], (int(boundRect[0][0]) - 15, int(centers[0][1]) - boundRect[0][3]), cv2.FONT_HERSHEY_SIMPLEX, 1, color, thickness)
            elif len(contours) > 1:
                bigger = 0
                length = 0
                for i in range(len(contours)):
                    if len(contours[i]) > length:
                        length = len(contours[i])
                        bigger = i
                cv2.rectangle(cv_image, (int(boundRect[bigger][0]), int(boundRect[bigger][1])), (int(boundRect[bigger][0] + boundRect[bigger][2]), int(boundRect[bigger][1] + boundRect[bigger][3])), color, thickness)
        # --------------------------------------------------------------------------------------

    def imageTreatment(self, msg, camera):
        # ---------------------------- Initialize the CvBridge class ---------------------------
        bridge = CvBridge()
        # --------------------------------------------------------------------------------------

        # ---------------- Try to convert the ROS Image message to a CV2 Image ----------------
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            if camera == 'Front':
                self.image_to_show = cv_image
            elif camera == 'Back':
                self.back_image_to_show = cv_image
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr(camera + ' camera Error!')
        # --------------------------------------------------------------------------------------

        # ----------------------------- Create players color masks -----------------------------
        green_mask = cv2.inRange(cv_image, (0, 100, 0), (50, 256, 50))
        red_mask = cv2.inRange(cv_image, (0, 0, 100), (50, 50, 256))
        blue_mask = cv2.inRange(cv_image, (100, 0, 0), (256, 50, 50))
        # --------------------------------------------------------------------------------------

        # ---------------- Morphological transformation for each mask - Closing ----------------
        kernel = np.ones((5, 5), np.uint8)  # Structuring element used for Closing.
        green_mask = cv2.dilate(green_mask, kernel, iterations=5)  # Morphological operator dilate applied to green mask.
        green_mask = cv2.erode(green_mask, kernel, iterations=5)  # Morphological operator erode applied to green mask.
        red_mask = cv2.dilate(red_mask, kernel, iterations=5)  # Morphological operator dilate applied to red mask.
        red_mask = cv2.erode(red_mask, kernel, iterations=5)  # Morphological operator erode applied to red mask.
        blue_mask = cv2.dilate(blue_mask, kernel, iterations=5)  # Morphological operator dilate applied to blue mask.
        blue_mask = cv2.erode(blue_mask, kernel, iterations=5)  # Morphological operator erode applied to blue mask.
        # --------------------------------------------------------------------------------------

        # --------- Attribute prey and hunter mask to each team according to his color ---------
        mask = []
        drawing_color = []
        if self.my_team == self.blue_team:
            prey_mask = red_mask
            hunter_mask = green_mask
            mask = [red_mask, green_mask]
            drawing_color = [(0, 0, 200), (0, 200, 0)]
        elif self.my_team == self.red_team:
            prey_mask = green_mask
            hunter_mask = blue_mask
            mask = [green_mask, blue_mask]
            drawing_color = [(0, 200, 0), (200, 0, 0)]
        elif self.my_team == self.green_team:
            prey_mask = blue_mask
            hunter_mask = red_mask
            mask = [blue_mask, red_mask]
            drawing_color = [(200, 0, 0), (0, 0, 200)]
        self.drawingFunction(cv_image, mask, drawing_color)
        # --------------------------------------------------------------------------------------

        # ----------------------------------- Prey detection -----------------------------------
        output_prey = cv2.connectedComponentsWithStats(prey_mask, connectivity=8)
        # # output_prey[0] -> nb_components; output_prey[1] -> output; output_prey[2] -> stats; output_prey[3] -> centroids

        biggest_prey = 0
        prey_idx = 0
        mass_center_prey = None

        # .................. Get prey center of mass ..................
        if output_prey[0] > 1:
            for i in range(1, output_prey[0]):
                if output_prey[2][i][4] > biggest_prey:
                    biggest_prey = output_prey[2][i][4]
                    prey_idx = i
            mass_center_prey = output_prey[3][prey_idx]
        # .............................................................
        # --------------------------------------------------------------------------------------

        # ---------------------------------- Hunter detection ----------------------------------
        output_hunter = cv2.connectedComponentsWithStats(hunter_mask, connectivity=8)

        biggest_hunter = 0
        hunter_idx = 0
        mass_center_hunter = None

        # ................. Get hunter center of mass .................
        if output_hunter[0] > 1:
            for i in range(1, output_hunter[0]):
                if output_hunter[2][i][4] > biggest_hunter:
                    biggest_hunter = output_hunter[2][i][4]
                    hunter_idx = i
            mass_center_hunter = output_hunter[3][hunter_idx]
        # .............................................................
        # --------------------------------------------------------------------------------------

        return mass_center_prey, mass_center_hunter, cv_image, prey_mask, hunter_mask

    # ------------------------------ Clustering Scan ------------------------------
    def CreateMarker(self):
        # Create marker
        marker = Marker()
        marker.header.frame_id = self.name + "/base_footprint"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.pose.orientation.w = 1.0  # Normalize quaternion
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 0.5
        marker.color.r = random.random()
        marker.color.g = random.random()
        marker.color.b = random.random()

        return marker
    # -----------------------------------------------------------------------------

    def laserScanMessageReceivedCallback(self, msg):
        x_prev, y_prev = 1000, 1000
        dist_threshold = 1.5

        marker_array = MarkerArray()

        z = 0
        all_body_dist = []

        for idx, rate in enumerate(msg.ranges):

            if rate < 0.1 or rate == math.inf:  # Take out all Infinite Markers or non-reflected laser.
                continue

            theta = msg.angle_min + msg.angle_increment * idx
            x = rate * math.cos(theta)
            y = rate * math.sin(theta)

            dist = math.sqrt((x_prev - x) ** 2 + (y_prev - y) ** 2)
            if dist > dist_threshold:  # New cluster
                marker = self.CreateMarker()
                idx_group = len(marker_array.markers)
                marker.id = idx_group
                marker.points = []
                marker_array.markers.append(marker)

            if len(marker_array.markers) > 0:
                last_marker = marker_array.markers[-1]
                last_marker.points.append(Point(x=x, y=y, z=0))

            x_prev = x
            y_prev = y

        publisher.publish(marker_array)

        if len(marker_array.markers) > 0:
            self.avoid_wall = False
            self.closer_body = False

            # ------------------------ Distinguish walls from bodies/robots  -----------------------
            for i in range(0, len(marker_array.markers)):
                if len(marker_array.markers[i].points) <= 2:                                # Insignificant objects detected.
                    pass
                elif len(marker_array.markers[i].points) >= 15:
                    self.wall_detected = True

                    # ................ Detect distance to wall ................
                    min_x_dist = marker_array.markers[i].points[0].x
                    min_y_dist = marker_array.markers[i].points[0].y
                    for ii in range(1, len(marker_array.markers[i].points)):
                        x_dist = marker_array.markers[i].points[ii].x
                        y_dist = marker_array.markers[i].points[ii].y
                        if x_dist < min_x_dist:
                            min_x_dist = x_dist
                        if y_dist < min_y_dist:
                            min_y_dist = y_dist

                    dist_to_wall = math.sqrt((min_x_dist ** 2) + (min_y_dist ** 2))

                    # .........................................................

                    if dist_to_wall < 1:
                        # ........... Variables needed to avoid walls .............
                        front_last = min(min(msg.ranges[342:360]), 10)
                        front_beggin = min(min(msg.ranges[0:18]), 10)

                        self.regions = {
                            # 'Right': min(min(msg.ranges[270:305]), 10),
                            'fRight': min(min(msg.ranges[320:341]), 10),
                            'Front': min(front_last, front_beggin),
                            'fLeft': min(min(msg.ranges[19:45]), 10),
                            # 'Left': min(min(msg.ranges[55:90]), 10),
                        }
                        # .........................................................
                        self.avoid_wall = True

                else:
                    # ................ Detect distance to body ................
                    min_x_dist = marker_array.markers[i].points[0].x
                    min_y_dist = marker_array.markers[i].points[0].y
                    for ii in range(1, len(marker_array.markers[i].points)):
                        x_dist = marker_array.markers[i].points[ii].x
                        y_dist = marker_array.markers[i].points[ii].y
                        if x_dist < min_x_dist:
                            min_x_dist = x_dist
                        if y_dist < min_y_dist:
                            min_y_dist = y_dist

                    dist_to_body = math.sqrt((min_x_dist ** 2) + (min_y_dist ** 2))

                    all_body_dist.append(dist_to_body)

                    self.closer_body_dist = min(all_body_dist)

                    # ______ Check if closer body it's a hunter ______
                    if (self.closer_body_dist <= 3.0) and (self.hunter_size > self.prey_size):
                        self.closer_body = True
                    # _______________________________________________
                    # .........................................................
            # --------------------------------------------------------------------------------------

    def avoidWall(self):
        self.robot_general_state = 'Avoiding walls'
        state_description = ''
        self.speed = 0.0
        self.angle = 0.0

        # -------------- Check wall position and set linear and angular velocity ---------------
        if self.regions['Front'] > 1 and self.regions['fLeft'] > 1 and self.regions['fRight'] > 1:
            state_description = 'Case 1 - Nothing'
            self.speed = 0.5
            self.angle = 0.0
        elif self.regions['Front'] < 1 and self.regions['fLeft'] > 1 and self.regions['fRight'] > 1:
            state_description = 'Case 2 - Front'
            self.speed = 0.5
            self.angle = 1.2
        elif self.regions['Front'] > 1 and self.regions['fLeft'] > 1 and self.regions['fRight'] < 1:
            state_description = 'Case 3 - Front Right'
            self.last_prey_detected = 'Left'
            self.speed = 0.5
            self.angle = 1.2
        elif self.regions['Front'] > 1 and self.regions['fLeft'] < 1 and self.regions['fRight'] > 1:
            state_description = 'Case 4 - Front Left'
            self.last_prey_detected = 'Right'
            self.speed = 0.5
            self.angle = -1.2
        elif self.regions['Front'] < 1 and self.regions['fLeft'] > 1 and self.regions['fRight'] < 1:
            state_description = 'Case 5 - Front and Front Right'
            self.last_prey_detected = 'Left'
            self.speed = 0.5
            self.angle = 1.2
        elif self.regions['Front'] < 1 and self.regions['fLeft'] < 1 and self.regions['fRight'] > 1:
            state_description = 'Case 6 - Front and Front Left'
            self.last_prey_detected = 'Right'
            self.speed = 0.5
            self.angle = -1.2
        elif self.regions['Front'] < 1 and self.regions['fLeft'] < 1 and self.regions['fRight'] < 1:
            state_description = 'Case 7 - Front, Front Left and Front Right'
            if self.regions['Front'] < 0.2 and self.regions['fLeft'] < 0.2 and self.regions['fRight'] < 0.2:
                self.speed = -1.0
                time.sleep(0.2)
                if self.regions['Front'] > 0.2 and self.regions['fLeft'] > 0.2 and self.regions['fRight'] > 0.2:
                    self.angle = 1.2
                else:
                    self.angle = -1.2
                time.sleep(0.2)
            else:
                self.speed = 0.5
                if self.regions['fRight'] < self.regions['fLeft']:
                    self.angle = 1.2
                else:
                    self.angle = -1.2
        elif self.regions['Front'] > 1 and self.regions['fLeft'] < 1 and self.regions['fRight'] < 1:
            state_description = 'Case 8 - Front Left and Front Right'
            self.speed = 1.0
            self.angle = 0.0
        else:
            state_description = 'Unknown case'

        time.sleep(0.3)
        rospy.loginfo(state_description)
        # --------------------------------------------------------------------------------------

    def catchPrey(self, minimum_speed=0.1, maximum_speed=1):
        # ------------------ Set linear and angular velocity to catch a prey -------------------
        self.robot_general_state = 'Catching prey'
        self.angle = -self.preyPosition / 400
        self.speed = 0.5
        self.counter = 0
        # --------------------------------------------------------------------------------------

    def avoidHunter(self, minimum_speed=0.1, maximum_speed=1):
        # ----------------- Set linear and angular velocity to avoid hunters -------------------
        self.robot_general_state = 'Running from hunter'
        self.angle = 0.0
        self.speed = 0.0

        if self.back_hunter_detected == True:
            self.angle = -self.back_hunterPosition / 400
            self.speed = 1 / self.closer_body_dist
        else:
            self.speed = 0.0
            if self.last_prey_detected == 'Right':
                self.angle = -4.5
            elif self.last_prey_detected == 'Left':
                self.angle = 4.5

        self.counter = 0
        # --------------------------------------------------------------------------------------

    def findPrey(self, minimum_speed=0.1, maximum_speed=1):
        # ------------------ Set linear and angular velocity to find preys ---------------------
        self.robot_general_state = "Finding for prey's"
        if self.counter <= 140:
            if self.last_prey_detected == 'Right':
                self.angle = -0.5
            elif self.last_prey_detected == 'Left':
                self.angle = 0.5
            self.speed = 0.0
            self.counter += 1
        elif 140 <= self.counter <= 160:
            self.angle = 0.0
            self.speed = 0.5
            self.counter += 1
        elif self.counter > 160:
            self.counter = 0
        # --------------------------------------------------------------------------------------

    # -------------------------- Drive though goal set manually ----------------------------
    def driveToGoal(self):
        goal_copy = copy.deepcopy(self.goal)
        goal_copy.header.stamp = rospy.Time.now()
        goal_in_base_lik = self.tf_buffer.transform(goal_copy, self.name + '/base_footprint', rospy.Duration(1))

        self.angle = math.atan2(goal_in_base_lik.pose.position.y, goal_in_base_lik.pose.position.x)

        # ......... Decrease linear velocity when it's closer to goal .........
        if 0.1 < goal_in_base_lik.pose.position.x <= 1.5:
            self.speed = goal_in_base_lik.pose.position.x
        else:
            self.speed = 0.5
        # .....................................................................

        # .................. Stop following the manual goal ...................
        if goal_in_base_lik.pose.position.x <= 0.1:
            self.manual_goal_active = False
        # .....................................................................
    # --------------------------------------------------------------------------------------

    def sendCommandCallback(self, event):
        # ------------------------------ Robot check what to do --------------------------------
        if self.manual_goal_active == True:
            self.driveToGoal()
        else:
            if ((self.hunter_detected == True) or (self.back_hunter_detected == True)) and (self.closer_body == True) and (self.avoid_wall == False):       # Avoid a hunter.
                self.avoidHunter()
            elif (self.prey_detected == False) and (self.hunter_detected == False) and (self.avoid_wall == False):                                          # No goal, find a prey.
                self.findPrey()
            elif (self.prey_detected == False) and (self.hunter_detected == False) and (self.avoid_wall == True):                                           # Avoid a wall.
                self.avoidWall()
            elif (self.prey_detected == True) and (self.hunter_detected == False) and (self.closer_body == False):                                          # Catch a prey.
                self.catchPrey()
        # --------------------------------------------------------------------------------------

        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.angle
        self.publisher_command.publish(twist)


def main():
    # rospy.init_node('p_bmendes_driver', anonymous=False)
    rospy.init_node('red1', anonymous=False)

    driver = Driver()
    rospy.spin()


if __name__ == '__main__':
    main()
