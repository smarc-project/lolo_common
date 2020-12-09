#!/usr/bin/env python

import rospy
from smarc_msgs.msg import LatLonOdometry, LatLonStamped, FloatStamped
from smarc_msgs.srv import LatLonToUTMOdometry
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
import math

class OldPoseConverter(object):

    def __init__(self):
        self.quat = None
        self.altitude = 0.
        self.pub = rospy.Publisher('dr/odom', Odometry, queue_size=10)
        self.quat_sub = rospy.Subscriber("core/state/orientation", QuaternionStamped, self.quat_callback)
        self.depth_sub = rospy.Subscriber("core/state/depth", FloatStamped, self.depth_callback)
        self.lat_lon_sub = rospy.Subscriber("core/state/position", LatLonStamped, self.lat_lon_callback)

    def quat_callback(self, msg):
        #print("Quat callback!")
        self.quat = msg.quaternion

    def depth_callback(self, msg):
        self.altitude = -msg.data

    def lat_lon_callback(self, msg):
        odom = LatLonOdometry()
        odom.lat_lon_pose.position.latitude = 180./math.pi*msg.latitude
        odom.lat_lon_pose.position.longitude = 180./math.pi*msg.longitude
        odom.lat_lon_pose.position.altitude = self.altitude

        print "Got ", msg.latitude, msg.longitude, self.altitude

        #print "Quat in lat lon: ", self.quat
        if self.quat is not None:
            odom.lat_lon_pose.orientation = self.quat
            #print "Adding odom: ", self.quat
        else:
            odom.lat_lon_pose.orientation.w = 1.

        try:
            translate_odom = rospy.ServiceProxy('lat_lon_to_utm_odom', LatLonToUTMOdometry)
            resp = translate_odom(odom)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return
        print "Got ", resp.odom.pose.pose.position
        self.pub.publish(resp.odom)

if __name__ == '__main__':
    rospy.init_node('old_pose_converter')
    conv = OldPoseConverter()
    rospy.spin()
