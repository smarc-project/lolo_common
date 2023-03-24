#! /usr/bin/env python3
# -*- coding: utf-8 -*-


"""
Since the INS handles all DR on Lolo, we need
a node that will publish what the DR stack used to
publish in SAM:
    for the BT:
        dr/lat_lon
    for nodered, in addition to BT stuff above:
        dr/
            altitude
            roll
            pitch
            yaw

there is some node in lolo_common that does roll/pitch/yaw from dr/odom,
but who publishes dr/odom in the new lolo?

# Ins message:
# int8 ALT_REF_GEOID=0
# int8 ALT_REF_ELLIPSOID=1
# std_msgs/Header header
  # uint32 seq
  # time stamp
  # string frame_id
# float64 latitude
# float64 longitude
# int8 altitude_ref
# float32 altitude
# float64[9] position_covariance
# float32 heading
# float32 roll
# float32 pitch
# float64[9] attitude_covariance
# geometry_msgs/Vector3 speed_vessel_frame
  # float64 x
  # float64 y
  # float64 z
# float64[9] speed_vessel_frame_covariance
"""

import rospy
# dr/lat_lon
from geographic_msgs.msg import GeoPoint
# dr/altitude, roll, pitch, yaw
from std_msgs.msg import Float64
# core/ins
from ixblue_ins_msgs.msg import Ins


class INSDr(object):
    def __init__(self,
                 ins_topic = "core/ins",
                 robot_name = "lolo"):
        self.ins_sub = rospy.Subscriber("/"+robot_name+"/"+ins_topic,
                                        Ins,
                                        self.ins_cb,
                                        queue_size=1)


        topic_root = "/"+robot_name+"/dr/"
        self.latlon_pub = rospy.Publisher(topic_root+"lat_lon", GeoPoint, queue_size=1)
        self.altitude_pub = rospy.Publisher(topic_root+"altitude", Float64, queue_size=1)
        self.roll_pub = rospy.Publisher(topic_root+"roll", Float64, queue_size=1)
        self.pitch_pub = rospy.Publisher(topic_root+"pitch", Float64, queue_size=1)
        self.yaw_pub = rospy.Publisher(topic_root+"yaw", Float64, queue_size=1)


    def ins_cb(self, msg):
        ll = GeoPoint()
        ll.latitude = msg.latitude
        ll.longitude = msg.longitude
        self.latlon_pub.publish(ll)
        self.altitude_pub.publish(msg.altitude)
        self.roll_pub.publish(msg.roll)
        self.pitch_pub.publish(msg.pitch)
        self.yaw_pub.publish(msg.heading)



if __name__ == "__main__":
    rospy.init_node("ins_to_dr")
    ins2dr = INSDr()
    while not rospy.is_shutdown():
        rospy.spin()























