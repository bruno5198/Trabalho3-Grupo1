#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
import argparse
from image_geometry.cameramodels import PinholeCameraModel
from dynamic_reconfigure.msg import ConfigDescription

global info

def callbackMsgReceived(marker_array):
    # rospy.loginfo('Receiving data')
    #
    # print(str(len(marker_array.markers)) + ' markers detected')

    pass

    # if len(marker_array.markers) > 0:
    #     print(marker_array.markers[0].points[0])
    #     print('\n\n\n\n')
    #
    #     for i in range(0, len(marker_array.markers)):
    #         if len(marker_array.markers[i].points) >= 15:
    #             print('marker ' + str(marker_array.markers[i].id) + ' is a wall')
    #         else:
    #             print('marker ' + str(marker_array.markers[i].id) + ' is a body')




def callbackCameraInfo(CameraInfo, publisher):
    # print(CameraInfo)

    CI = CameraInfo

    cam = PinholeCameraModel()
    cam.fromCameraInfo(CI)
    pix = cam.project3dToPixel((-0.12,1,0.2))

    # print(cam.project3dToPixel((0,0,0)))


    Pixel = Point()
    Pixel.x = int(round(pix[0]))
    Pixel.y = int(round(pix[1]))
    Pixel.z = 0

    print(Pixel)

    publisher.publish(Pixel)


def listener():
    # -------------------------------------
    # INITIALIZATION
    # -------------------------------------

    rospy.init_node('listener', anonymous=True)

    publisher = rospy.Publisher('/blue1/pixels', Point, )

    rospy.Subscriber('/blue1/markers', MarkerArray, callbackMsgReceived)
    rospy.Subscriber('blue1/camera/rgb/camera_info', CameraInfo, callbackCameraInfo, publisher)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()