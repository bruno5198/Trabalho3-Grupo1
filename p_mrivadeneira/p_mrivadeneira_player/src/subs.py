#!/usr/bin/env python3
import geometry_msgs.msg
import rospy
import tf
import tf2_ros
import numpy as np
import cv2
import re
import tf2_ros
from geometry_msgs.msg import Twist, Pose, PoseStamped
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import CameraInfo
from image_geometry.cameramodels import PinholeCameraModel
from p_mrivadeneira_player.msg import camera_detection

import copy
from math import *

class Driver():

    def __init__(self):

        # Player's name
        self.name = rospy.get_name()
        self.name = self.name.strip('/')        # Remove initial '/'

        # Command Velocity
        # self.publisher_command = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.SendCommandCallback)

        # Subscriptions
        self.scan_subscriber = rospy.Subscriber(self.name + '/markers', MarkerArray, self.LaserScanPointsReceived)
        self.camera_info = rospy.Subscriber(self.name + '/camera/rgb/camera_info', CameraInfo, self.CameraInfoReceived)
        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.GoalReceivedCallback)


        # self.listener = tf.TransformListener



        self.goal = PoseStamped()
        self.goal_active = False

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def LaserScanPointsReceived(self, msg):

        # print(msg)

        # for i in range(0, len(msg.markers)):
        #     for k in range(0, len(msg.markers[i].points)):          # Transformation from base_scan to camera_rgb_frame ==> x=-0.029; y=0, z=0.14
        #         msg.markers[i].points[k].x = msg.markers[i].points[k].x-0.029
        #         msg.markers[i].points[k].z = msg.markers[i].points[k].z + 0.14

        # print(msg)

        self.markers = msg



        now = rospy.Time.now()
        trans = self.tf_buffer.lookup_transform("blue1/camera_rgb_optical_frame", "blue1/base_scan", now)

        # print(trans)

        # point = Pose()
        # point.position.x = msg.markers[0].points[0].x
        # point.position.y = msg.markers[0].points[0].y
        # point.position.z = msg.markers[0].points[0].z
        # point.orientation.x = 0
        # point.orientation.y = 0
        # point.orientation.z = 0
        # point.orientation.w = 1

        pose = geometry_msgs.msg.PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "blue1/base_scan"
        pose.pose.position.x = 3
        pose.pose.position.y = 0
        pose.pose.position.z = 0
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1

        # new_point = self.tfBuffer.transform(pose, "blue1/camera_rgb_optical_frame", rospy.Duration(1))
        #
        # print(new_point)

    def GoalReceivedCallback(self, goal):

        goal_copy = copy.deepcopy(self.goal)
        goal_copy.header.stamp = rospy.Time.now()

        print('Transforming pose')
        print(goal_copy)
        goal_in_base_link = self.tf_buffer.transform(goal_copy, self.name + '/camera_rgb_optical_frame', rospy.Duration(1))
        print('Pose transformed')

        y = goal_in_base_link.pose.position.y
        x = goal_in_base_link.pose.position.x

    def CameraInfoReceived(self, CI):

        # print(CI)

        self.cam_info = CI
        # ci = CameraInfo()
        # ci.width = 1200
        # ci.height = 380

        cam = PinholeCameraModel()
        cam.fromCameraInfo(CI)

        for i in range(0, len(self.markers.markers)):
            for k in range(0, len(self.markers.markers[i].points)):
                x = self.markers.markers[i].points[k].x
                y = self.markers.markers[i].points[k].y
                z = self.markers.markers[i].points[k].z
                pix = cam.project3dToPixel((y, z, x))
                # pix = cam.project3dToPixel((0, 10, 0))
                # if pix[0] > 0 and pix[1] < 380 and pix[1] > 0:
                # print(self.markers.markers[i].points[k])
                # print(pix)
                # print('\n\n')


    def SendCommandCallback(self, event):

        print('Sending Twist Command!')


def talker():

    # -------------------------------------
    # INITIALIZATION
    #-------------------------------------

    rospy.init_node('p_mrivadeneira', anonymous=False)
    driver = Driver()

    rate = rospy.Rate(10)

    rospy.spin()
    # -------------------------------------
    # EXECUTION
    # -------------------------------------

    while not rospy.is_shutdown():

        pass
    # -------------------------------------
    # TERMINATION
    # -------------------------------------
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass