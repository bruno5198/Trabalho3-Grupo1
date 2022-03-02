#!/usr/bin/env python3
import random

import rospy
import re
import sys
import tf2_ros
import time
from math import *
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Float64
from p_mrivadeneira_player.msg import DetectionInfo


class Driver():

    def __init__(self):

        # Player's name
        self.name = rospy.get_name()
        self.name = self.name.strip('/')        # Remove initial '/'

        # Command Velocity
        self.publisher_command = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.SendCommandCallback)

        # Subscriptions
        self.coords_subscriber = rospy.Subscriber(self.name + '/detection', DetectionInfo, self.SensorsCallback)
        self.left_arm_subscriber = rospy.Subscriber(self.name + '/left_arm_base_to_left_arm_controller/command', Float64, self.left_arm)
        self.right_arm_subscriber = rospy.Subscriber(self.name + '/right_arm_base_to_right_arm_controller/command', Float64, self.right_arm)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Message variables initialization --------------------------------------------------
        # Goal
        self.flag_goal_active = False
        self.flag_goal_achieved = False
        self.goal = Point()

        # Target
        self.flag_target_close = False
        self.target_center = 0
        self.target_area = 0
        self.target = Point()
        self.last_target_position_left = False

        # Threat
        self.flag_threat_close = False
        self.threat_center = 0
        self.threat_area = 0
        self.threat = Point()
        self.last_threat_position_left = True

        # Near bodies
        self.flag_near_body = False
        self.near_body_position = 0

        # Near walls
        self.flag_near_wall = False
        self.near_wall_coords = (0,0)
        self.walls = [False, False, False, False]
        self.previous_front_wall = False

        # State machine
        self.state = 'None'

        # Initial values
        self.angle = 0
        self.speed = 0

        # Arms position
        self.left_arm_position = 0
        self.right_arm_position = 0

        # Interface
        args = rospy.myargv(argv=sys.argv)
        try:
            self.visualize = args[1]
            self.run = args[2]
        except:
            self.visualize = "false"
            self.run = "true"

        self.TeamsDefinition(self.name)

    def left_arm(self, pos):

        self.left_arm_position = pos.data

    def right_arm(self, pos):

        self.right_arm_position = pos.data

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

            if self.visualize == 'true':
                print('My name is: ' + self.name + ". I am team " + self.my_team + '. I am hunting ' + str(preys) + ' and fleeing from ' + str(hunts))
        except:
            if self.visualize == 'true':
                print("I can't understand the color of my team. I'm gonna die alone. I'm sad. :(")

    def Interface(self):

        print('----------------------------------------------------')
        print('DRIVER NODE')
        print('Name: ' + self.name)
        print('State: ' + self.state + '\n')

        print('Detection information:')
        print('     GOAL')
        print('         Goal detection: ' + str(self.flag_goal_active))
        print('         Goal achieved: ' + str(self.flag_goal_achieved))
        print('         Goal position: ' + str((round(self.goal.x,3), round(self.goal.y,3))))

        print('     BODY')
        print('         Body near: ' + str(self.flag_near_body))

        print('     Target')
        print('         Target center: ' + str(self.target_center))
        print('         Target near: ' + str(self.flag_target_close))
        print('         Target position: ' + str((round(self.target.x,3), round(self.target.y,3))))

        print('     Threat')
        print('         Threat center: ' + str(self.threat_center))
        print('         Threat near: ' + str(self.flag_threat_close))
        print('         Threat position: ' + str((round(self.threat.x,3), round(self.threat.y,3))))

        print('     Walls')
        print('         [RIGHT FRONT LEFT BACK FRONT(collision)]: ' + str(self.walls))

        print('\n')
        print('Velocity:')
        print('     Linear: ' + str(round(self.speed,3)))
        print('     Angular: ' + str(round(self.angle,3)))

        if self.right_arm_position == float(0.0):
            print('\n')
            print('Arms position: CLOSED')
        elif self.right_arm_position >= float(1.0):
            print('\n')
            print('Arms position: OPEN (ATTACK)')
        else:
            print('\n')
            print('Arms position: OPEN (DEFENCE)')


    def SensorsCallback(self, detection_info):

        # print('The detection info is: ')
        # print(detection_info)
        # print('\n\n')

        # Update message variables --------------------------------------------------
        # Goal
        self.flag_goal_active = detection_info.flag_goal_active
        self.flag_goal_achieved = detection_info.flag_goal_achieved
        self.goal = detection_info.goal

        # Body
        self.flag_near_body = detection_info.flag_near_body

        # Target
        self.flag_target_close = detection_info.flag_target_close
        self.target_center = detection_info.target_center
        self.target_area = detection_info.target_area
        self.target = detection_info.target

        # Threat
        self.flag_threat_close = detection_info.flag_threat_close
        self.threat_center = detection_info.threat_center
        self.threat_area = detection_info.threat_area
        self.threat = detection_info.threat

        # Walls
        self.walls = detection_info.walls

        # Define the state machine

        if self.flag_target_close:
            self.state = 'Catching near target'
        elif self.flag_threat_close:
            self.state = 'Escaping from near threat'
        else:
            if self.target_center == 0 and self.threat_center == 0 and not self.flag_near_body:
                self.state = 'Wondering'
            elif self.target_center == 0 and self.threat_center == 0 and self.flag_near_body:
                self.state = 'Running away from unknown robot'
            elif (self.target_center != 0 and self.threat_center == 0) or (self.target_center != 0 and self.threat_center != 0 and self.target_area >= self.threat_area):
                self.state = 'Following target'
            elif self.target_center != 0 and self.threat_center != 0 and self.target_area <= self.threat_area:
                self.state = 'Following target avoiding threat'
            elif self.target_center == 0 and self.threat_center != 0:
                self.state = 'Avoiding threat'

        if self.flag_goal_active:
            self.state = 'Following Goal'

        if self.walls[4]:
            self.state = 'Reverse Gear'

    # def LaserScanPointsReceived(self, marker_array):
    #
    #     self.flag_near_body = False
    #     self.flag_near_wall = False
    #     self.near_wall_coords = (0, 0)
    #
    #     # print('\n\nNew measure: ')
    #
    #     if len(marker_array.markers) > 0:
    #         # print(marker_array.markers[0].points[0])
    #         # print('\n\n\n\n')
    #
    #         for i in range(0, len(marker_array.markers)):
    #             if len(marker_array.markers[i].points) <= 2:
    #                 pass
    #             elif len(marker_array.markers[i].points) >= 15:
    #                 # print('marker ' + str(marker_array.markers[i].id) + ' is a wall')
    #                 pass
    #             else:
    #                 # print('marker ' + str(marker_array.markers[i].id) + ' is a body')
    #                 self.flag_near_body = True

    def Driver(self):


        lin = self.speed
        ang = self.angle
        target_position = self.target_center
        target_area = self.target_area
        threat_position = self.threat_center
        threat_area = self.threat_area

        max_linear = 1.5
        min_linear = 0.25
        error_ang = 0
        error_ang_prev = 0
        kp = 0.001
        kd = 0.000125

        # Walls
        Z0 = self.walls[0]
        Z1 = self.walls[1]
        Z2 = self.walls[2]
        Z3 = self.walls[3]

        if self.state == 'Wondering':

            # lin = 0
            # if self.last_target_position_left:
            #     ang = 0.7
            # else:
            #     ang = -0.7
            #
            # if self.walls[3]:
            #     ang = -ang
            #
            #     if not self.previous_front_wall:
            #         self.last_target_position_left = not self.last_target_position_left         # In order to change the rotation when a wall is in front, so don't waste tine
            #
            # self.previous_front_wall = copy.deepcopy(self.walls[3])

            if not Z1:
                lin = 0.7
                ang = 0
            else:
                lin = 0.3
                if Z0:
                    ang = 1
                elif Z2:
                    ang = -1
                else:
                    choises = [-1, 1]
                    k = random.choice(choises)
                    k = 1
                    ang = k*0.5

        if self.state == 'Running away from unknown robot':

            if not Z1:
                lin = 1
                ang = 0
            else:
                lin = 1
                if Z0:
                    ang = 0.5
                else:
                    ang = -0.5

        if self.state == 'Following target':

            lin += 0.02

            if lin > max_linear:  # Saturate linear velocity
                lin = max_linear
            if lin < min_linear:
                lin = min_linear

            error_ang = (600 - target_position)
            ang = kp * error_ang + kd * (error_ang - error_ang_prev) / 2    # Angular value PD controlled - Better results agains P controller

            if target_position <= 600:  # Remember the last side where the target was
                self.last_target_position_left = True
            else:
                self.last_target_position_left = False

        if self.state == 'Catching near target':

            x = self.target.x
            y = self.target.y

            distance_to_goal = sqrt(x ** 2 + y ** 2) / 2.5

            ang = atan2(y, x)

            G = 1
            m = 0.5

            lin = G*m/(distance_to_goal + 0.001)  # Gravitational Proportional Controller

        if self.state == 'Following target avoiding threat':

            lin += 0.02

            if lin > max_linear:  # Saturate linear velocity
                lin = max_linear
            if lin < min_linear:
                lin = min_linear

            if threat_position <= 600:
                error_ang = (600 - target_position) - threat_position/2
            else:
                error_ang = (600 - target_position) + threat_position/2

            ang = kp * error_ang + kd * (error_ang - error_ang_prev) / 2  # Angular value PD controlled - Better results agains P controller

            if target_position <= 600:  # Remember the last side where the target was
                self.last_target_position_left = True
            else:
                self.last_target_position_left = False

        if self.state == 'Avoiding threat':

            lin += 0.02

            if lin > max_linear:  # Saturate linear velocity
                lin = max_linear
            if lin < min_linear:
                lin = min_linear

            # if threat_position <= 600:
            error_ang = (300 - threat_position)
            # else:
            #     error_ang = (600 - target_position) + threat_position / 2

            ang = kp * error_ang + kd * (error_ang - error_ang_prev) / 2  # Angular value PD controlled - Better results agains P controller

        if self.state == 'Following Goal':

            x = self.goal.x
            y = self.goal.y

            distance_to_goal = sqrt(x ** 2 + y ** 2)/2.5

            ang = atan2(y,x)
            lin = 0.5

            # lin = 0.5/(distance_to_goal + 0.001)          # Gravitational Proporcional Controller

        if self.state == 'Reverse Gear':
            lin = -1
            ang = 0
            time.sleep(0.2)

        # Run driver
        if self.run == 'true':
            self.speed = lin
            self.angle = ang
        else:
            self.speed = 0
            self.angle = 0

    def SendCommandCallback(self, event):

        if self.visualize == 'true':
            self.Interface()

        self.Driver()

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