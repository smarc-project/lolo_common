#!/usr/bin/python
import rospy
from std_msgs.msg import Header
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix

from smarc_msgs.msg import CaptainStatus
from imc_ros_bridge.msg import VehicleState

import math
import numpy as np


class translator:
    def __init__(self):

        #Heartbeat
        self.heartbeat_publisher = rospy.Publisher("/lolo/Heartbeat", Empty, queue_size=1)

        #State
        self.state_publisher = rospy.Publisher("/lolo/lolo/estimated_state", Pose, queue_size=1)
        self.state_subscriber = rospy.Subscriber("/lolo/core/state/position", PoseWithCovarianceStamped, self.callback_state)

        #Gps
        self.gps_publisher_fix = rospy.Publisher("/lolo/lolo/gps_fix", NavSatFix, queue_size=1)
        self.gps_publisher_nav = rospy.Publisher("/lolo/lolo/gps_nav_data", NavSatFix, queue_size=1)
        self.gps_subscriber = rospy.Subscriber("/lolo/core/gps", NavSatFix, self.callback_gps)

        #VehicleState
        self.vehiclestate_publisher = rospy.Publisher("/lolo/lolo/vehicle_state", VehicleState, queue_size=1)
        self.captainstatus_subscriber = rospy.Subscriber("/lolo/core/control_status", CaptainStatus, self.callback_captainstatus)

    def callback_state(self,msg):
        print("new state")
        newMsg = Pose()
        newMsg.position.x = msg.pose.pose.position.x
        newMsg.position.y = msg.pose.pose.position.y
        newMsg.position.z = msg.pose.pose.position.z
        newMsg.orientation = msg.pose.pose.orientation
        self.state_publisher.publish(newMsg)

    def callback_gps(self, msg):
        print("new GPS")
        self.gps_publisher_fix.publish(msg)
        self.gps_publisher_nav.publish(msg)

    def callback_captainstatus(self,msg):
      print("new captain status")
      vehiclestate_msg = VehicleState()
      vehiclestate_msg.op_mode = vehiclestate_msg.SERVICE;
      if(msg.active_control_input == 4): vehiclestate_msg.op_mode = vehiclestate_msg.ERROR;
      vehiclestate_msg.error_count = 0
      #vehiclestate_msg.error_ents = ;
      #vehiclestate_msg.maneuver_type =
      #vehiclestate_msg.maneuver_stime =
      #vehiclestate_msg.maneuver_eta =
      #vehiclestate_msg.flags =
      #vehiclestate_msg.last_error =
      vehiclestate_msg.control_loops = 0; #TODO find out how this is used
      #vehiclestate_msg.last_error_time =
      self.vehiclestate_publisher.publish(vehiclestate_msg)
      


def main():
    print("Starting")
    rospy.init_node('lolo_translation_node', anonymous=False)

    tr = translator();

    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        tr.heartbeat_publisher.publish() # TODO this should not be here!
        rate.sleep()

if __name__ == "__main__":
    main()
