#!/usr/bin/env python3
import argparse

import rospy
import re
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
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

        # Setpoint / Goal
        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.GoalReceivedCallback)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.goal = PoseStamped()
        self.goal_active = False

        # Initial values
        self.angle = 0
        self.speed = 0

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

    def DriveStraight(self, min_speed=0.3, max_speed=1):

        goal_copy = copy.deepcopy(self.goal)
        goal_copy.header.stamp = rospy.Time.now()

        # goal_tf = tf2_geometry_msgs.PoseStamped()
        # goal_tf.header.stamp = rospy.Time.now()
        # goal_tf.header.frame_id = self.goal.header.frame_id
        print('Transforming pose')
        print(goal_copy)
        goal_in_base_link = self.tf_buffer.transform(goal_copy, self.name + '/base_footprint', rospy.Duration(1))
        print('Pose transformed')

        y = goal_in_base_link.pose.position.y
        x = goal_in_base_link.pose.position.x

        distance_to_goal = sqrt(x ** 2 + y ** 2)

        if distance_to_goal >= 0.1:
            self.angle = atan2(y,x)

            self.speed = max(min_speed, 0.5*distance_to_goal)        # Linear velocity limited for minimum
            self.speed = min(max_speed, self.speed)                   # Linear velocity limited for maximum
        else:
            # self.angle = 0
            # self.speed = 0
            print('Goal achieved!')
            self.goal_active = False

    def GoalReceivedCallback(self, msg):

        print('New Goal Received')
        target_frame = self.name + '/odom'

        try:
            self.goal = self.tf_buffer.transform(msg, target_frame, rospy.Duration(1))
            self.goal_active = True
            rospy.logwarn('Setting new goal')
        except:
            self.goal_active = False
            rospy.logwarn('Could not transform goal from ' + msg.header.frame_id + ' to ' + target_frame)

    def SendCommandCallback(self, event):
        print('Sending Twist Command!')

        if not self.goal_active:
            self.speed = 0
            self.angle = 0
        else:
            self.DriveStraight()

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