#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from p_mrivadeneira_player.msg import DetectionInfo

global pub_left
global pub_left2
global pub_right
global pub_right2

def SensorsCallback(msg):

    global pub_left
    global pub_left2
    global pub_right
    global pub_right2

    target = msg.flag_target_close
    threat = msg.flag_near_body


    if target:
        value = 1.8
        value2 = -2
    elif threat:
        value = 0.9
        value2 = 1.48
    else:
        value = 0.0
        value2 = 0

    pub_left.publish(-value)
    pub_right.publish(value)
    pub_left2.publish(-value2)
    pub_right2.publish(value2)

def main():

    global pub_left
    global pub_left2
    global pub_right
    global pub_right2

    rospy.init_node('arms_control', anonymous=False)
    rate = rospy.Rate(10)

    rospy.Subscriber('/detection', DetectionInfo, SensorsCallback)
    pub_left = rospy.Publisher('/left_arm_base_to_left_arm_controller/command', Float64, queue_size=10)
    pub_left2 = rospy.Publisher('/left_arm_2_base_to_left_arm_2_controller/command', Float64, queue_size=10)
    pub_right = rospy.Publisher('/right_arm_base_to_right_arm_controller/command', Float64, queue_size=10)
    pub_right2 = rospy.Publisher('/right_arm_2_base_to_right_arm_2_controller/command', Float64, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    while not rospy.is_shutdown():

        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass