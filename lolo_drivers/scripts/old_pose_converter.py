#!/usr/bin/env python

import rospy
from smarc_msgs.msg import LatLonOdometry, LatLonStamped, FloatStamped
from smarc_msgs.srv import LatLonToUTMOdometry
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry

quat = None
altitude = 0.
pub = None

def quat_callback(msg):
    quat = msg.quaternion

def depth_callback(msg):
    altitude = -msg.data

def lat_lon_callback(msg):
    odom = LatLonOdometry()
    odom.lat_lon_pose.position.latitude = msg.latitude
    odom.lat_lon_pose.position.longitude = msg.longitude
    odom.lat_lon_pose.position.altitude = altitude
    if quat is not None:
        odom.lat_lon_pose.orientation = quat
    try:
        translate_odom = rospy.ServiceProxy('lat_lon_to_utm_odom', LatLonToUTMOdometry)
        resp = translate_odom(odom)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return
    pub.publish(resp.odom)

if __name__ == '__main__':
    rospy.init_node('old_pose_converter')
    pub = rospy.Publisher('dr/odom', Odometry, queue_size=10)
    quat_sub = rospy.Subscriber("core/state/orientation", QuaternionStamped, quat_callback)
    depth_sub = rospy.Subscriber("core/state/depth", FloatStamped, depth_callback)
    lat_lon_sub = rospy.Subscriber("core/state/position", LatLonStamped, lat_lon_callback)
    rospy.spin()
