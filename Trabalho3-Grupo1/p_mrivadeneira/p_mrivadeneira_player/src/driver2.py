#!/usr/bin/env python3

import rospy
import re
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import MarkerArray
from p_mrivadeneira_player.msg import camera_detection

import copy
from math import *

class Driver():

    def __init__(self):

        # Player's name
        self.name = rospy.get_name()
        self.name = self.name.strip('/')        # Remove initial '/'

        self.TeamsDefinition(self.name)

        # Command Velocity
        self.publisher_command = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.SendCommandCallback)

        # Subscriptions
        self.coords_subscriber = rospy.Subscriber(self.name + '/setpoint_msg/coords', camera_detection, self.CoordsReceivedCallback)
        self.scan_subscriber = rospy.Subscriber(self.name + '/markers', MarkerArray, self.LaserScanPointsReceived)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.goal = PoseStamped()
        self.goal_active = False

        # Initial values
        self.angle = 0
        self.speed = 0

        self.target_position = 0
        self.target_area = 0
        self.last_target_position_left = True
        self.threat_position = 0
        self.threat_area = 0

        self.flag_near_body = False
        self.near_body_position = 0

        self.flag_near_wall = False
        self.near_wall_coords = (0,0)

    def TeamsDefinition(self, name):

        self.my_team = re.sub('[0-9]+', '', name)

        try:
            if self.my_team == 'blue':
                self.prey_team = 'red'
                self.hunter_team = 'green'
            elif self.my_team == 'red':
                self.prey_team = 'green'
                self.hunter_team = 'blue'
            elif self.my_team == 'green':
                self.prey_team = 'blue'
                self.hunter_team = 'red'

            preys = rospy.get_param(self.prey_team + '_players')
            hunts = rospy.get_param(self.hunter_team + '_players')

            print('My name is: ' + self.name + ". I am team " + self.my_team + '. I am hunting ' + str(preys) + ' and fleeing from ' + str(hunts))
        except:
            print("I can't understand the color of my team. I'm gonna die alone. I'm sad. :(")

    def CoordsReceivedCallback(self, cam_detection):

        # print('The readed coords are: ')
        # print(cam_detection)

        self.target_position = cam_detection.target_center
        self.target_area = cam_detection.target_area
        self.threat_position = cam_detection.threat_center
        self.threat_area = cam_detection.threat_area

    def LaserScanPointsReceived(self, marker_array):

        self.flag_near_body = False
        self.flag_near_wall = False
        self.near_wall_coords = (0, 0)

        print('\n\nNew measure: ')

        if len(marker_array.markers) > 0:
            # print(marker_array.markers[0].points[0])
            # print('\n\n\n\n')

            for i in range(0, len(marker_array.markers)):
                if len(marker_array.markers[i].points) <= 2:
                    pass
                elif len(marker_array.markers[i].points) >= 15:
                    print('marker ' + str(marker_array.markers[i].id) + ' is a wall')
                else:
                    print('marker ' + str(marker_array.markers[i].id) + ' is a body')
                    self.flag_near_body = True


    def Driver(self, target_position, target_area, threat_position, threat_area, near_body):


        lin = self.speed
        ang = self.angle

        max_linear = 2
        min_linear = 0.25
        error_ang = 0
        error_ang_prev = 0
        kp = 0.0005
        kd = 0.00025

        # -------------------------------------
        # RUNAWAY MODE
        # -------------------------------------

        # if near_body and target_position == 0 and threat_position == 0:
        #     lin = 2.5
        # if not near_body:
        #     lin = 0


        # -------------------------------------
        # ATTACK MODE
        # -------------------------------------

        if target_position == 0:            # There is no target, so continue rotating until you get a target
            lin = 0
            if self.last_target_position_left == True:      # If the last target was at the right, then rotate right. Otherwise, rotate left.
                ang = 1
            else:
                ang = -1

        else:
            lin += 0.025

            if lin > max_linear:            # Saturate velocity
                lin = max_linear
            if lin < min_linear:
                lin = min_linear

            error_ang = (600 - target_position)
            if abs(error_ang) > 585:        # Reduction of rotational value when target founded
                lin = 0
                # if abs(ang) > 0.4:
                if error_ang > 0:
                    ang = 0.25
                else:
                    ang = -0.25
            else:
                ang = kp*error_ang + kd*(error_ang - error_ang_prev)/2      # Angular value PD controlled

            if target_position <= 600:           # Remember the last side where the target was
                self.last_target_position_left = True
            else:
                self.last_target_position_left = False

        # -------------------------------------
        # BREATHING MODE
        # -------------------------------------


        # if not self.flag_near_wall:
        #     lin = 1
        #     ang = 0
        # else:
        #     lin = 0
        #     # if self.near_wall_coords[0] < -0.1:
        #     if self.near_wall_coords[1] > 0:
        #         ang = -0.5
        #     else:
        #         ang = 0.5




        print('My velocity is: lin = ' + str(lin) + ' rot = ' + str(ang))
        self.speed = lin
        self.angle = ang


    def SendCommandCallback(self, event):
        # print('Sending Twist Command!')

        # if self.target_position != 0:
        #
        #     self.speed = 1
        #     self.angle = (600 - self.target_position)*0.005
        # else:
        #     self.speed = 0
        #     self.angle = 0.5

        self.Driver(self.target_position, self.target_area, self.threat_position, self.threat_area, self.flag_near_body)


        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.angle

        self.publisher_command.publish(twist)

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