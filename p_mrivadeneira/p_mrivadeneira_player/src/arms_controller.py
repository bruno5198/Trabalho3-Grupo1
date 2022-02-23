#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from p_mrivadeneira_player.msg import detection_info

global pub_left
global pub_right

def SensorsCallback(msg):

    global pub_left
    global pub_right

    target = msg.flag_target_close

    if target:
        value = 1.8
    else:
        value = 0.0

    pub_left.publish(-value)
    pub_right.publish(value)

def main():

    global pub_left
    global pub_right

    rospy.init_node('arms_control', anonymous=False)
    rate = rospy.Rate(10)

    rospy.Subscriber('/detection', detection_info, SensorsCallback)
    pub_left = rospy.Publisher('/left_arm_base_to_left_arm_controller/command', Float64, queue_size=10)
    pub_right = rospy.Publisher('/right_arm_base_to_right_arm_controller/command', Float64, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    while not rospy.is_shutdown():

        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass