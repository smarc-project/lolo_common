#!/usr/bin/python

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
import numpy as np

class hejsan:
    def __init__(self):
        self.heartbeat_publisher = rospy.Publisher("/lolo/Heartbeat", Empty, queue_size=1)

        #State
        self.state_publisher = rospy.Publisher("/lolo/lolo/estimated_state", Pose, queue_size=1)
        self.state_subscriber = rospy.Subscriber("/lolo/state/position", PoseWithCovarianceStamped, self.callback_state)

        #Gps
        self.gps_publisher_fix = rospy.Publisher("/lolo/lolo/gps_fix", NavSatFix, queue_size=1)
        self.gps_publisher_nav = rospy.Publisher("/lolo/lolo/gps_nav_data", NavSatFix, queue_size=1)
        self.gps_subscriber = rospy.Subscriber("/lolo/sensors/gps", NavSatFix, self.callback_gps)

    def callback_state(self,msg):
        print("new state")
        newMsg = Pose()
        #print(msg.pose.pose.position.x)
        #print(newMsg.position.x)
        newMsg.position.x = msg.pose.pose.position.x
        newMsg.position.y = msg.pose.pose.position.y
        newMsg.position.z = msg.pose.pose.position.z
        newMsg.orientation = msg.pose.pose.orientation
        self.state_publisher.publish(newMsg)

    def callback_gps(self, msg):
        print("new GPS")
        self.gps_publisher_fix.publish(msg)
        self.gps_publisher_nav.publish(msg)


def main():
    print("Starting")
    rospy.init_node('loloTest', anonymous=False)

    hej = hejsan();



    rate = rospy.Rate(1) # 1hz
    t = 0.
    while not rospy.is_shutdown():
        hej.heartbeat_publisher.publish()
        rate.sleep()

if __name__ == "__main__":
    main()
