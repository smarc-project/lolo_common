#!/usr/bin/python
import rospy
from std_msgs.msg import Header
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix
import tf

from captain_interface.msg import CaptainStatus
from smarc_msgs.msg import Float32Stamped

from imc_ros_bridge.msg import VehicleState
from imc_ros_bridge.msg import EstimatedState
from imc_ros_bridge.msg import DesiredHeading
from imc_ros_bridge.msg import DesiredHeadingRate
from imc_ros_bridge.msg import DesiredPitch
from imc_ros_bridge.msg import DesiredRoll
from imc_ros_bridge.msg import DesiredSpeed
from imc_ros_bridge.msg import DesiredZ
from imc_ros_bridge.msg import RemoteState

import math
import numpy as np


class translator:
    class AUV:
        def __init__(self):
            self.lat = 0
            self.lon = 0
            self.depth = 0
            self.altitude = 0
            self.pitch = 0
            self.roll = 0
            self.yaw = 0
            self.vx = 0
            self.vy = 0
            self.vz = 0
            self.accX = 0
            self.accY = 0
            self.accZ = 0
            self.rotX = 0
            self.rotY = 0
            self.rotZ = 0
        
        

    def __init__(self):
        self.lolo = self.AUV()

        #Heartbeat
        self.heartbeat_publisher = rospy.Publisher("/lolo/Heartbeat", Empty, queue_size=1)

        #State
        self.state_publisher = rospy.Publisher("/lolo/imc/estimated_state", EstimatedState, queue_size=1)
        self.state_pos_subscriber = rospy.Subscriber("/lolo/core/state/position", PoseWithCovarianceStamped, self.callback_state_pos)
        self.state_twist_subscriber = rospy.Subscriber("/lolo/core/state/twist", TwistWithCovarianceStamped, self.callback_state_twist)
        self.state_altitude_subscriber = rospy.Subscriber("/lolo/core/state/altitude", Float32Stamped, self.callback_state_altitude)

        #remote state
        self.remote_state_publisher = rospy.Publisher("lolo/imc/remote_state", RemoteState, queue_size=1)

        #Gps
        self.gps_publisher_fix = rospy.Publisher("/lolo/imc/gps_fix", NavSatFix, queue_size=1)
        self.gps_publisher_nav = rospy.Publisher("/lolo/imc/gps_nav_data", NavSatFix, queue_size=1)
        self.gps_subscriber = rospy.Subscriber("/lolo/core/gps", NavSatFix, self.callback_gps)

        #VehicleState
        self.vehiclestate_publisher = rospy.Publisher("/lolo/imc/vehicle_state", VehicleState, queue_size=1)
        self.captainstatus_subscriber = rospy.Subscriber("/lolo/core/control_status", CaptainStatus, self.callback_captainstatus)

        #Controller setpoints
        self.targetYaw_publisher = rospy.Publisher("lolo/imc/desired_heading", DesiredHeading, queue_size=1)
        self.targetPitch_publisher = rospy.Publisher("lolo/imc/desired_pitch", DesiredPitch, queue_size=1)
        self.targetSpeed_publisher = rospy.Publisher("lolo/imc/desired_speed", DesiredSpeed, queue_size=1)
        self.targetDepth_publisher = rospy.Publisher("lolo/imc/desired_z", DesiredZ, queue_size=1)

    def callback_state_twist(self,msg):
        self.lolo.vx = msg.twist.twist.linear.x
        self.lolo.vy = msg.twist.twist.linear.y
        self.lolo.vz = msg.twist.twist.linear.z
        self.lolo.rotX = msg.twist.twist.angular.x
        self.lolo.rotY = msg.twist.twist.angular.y
        self.lolo.rotZ = msg.twist.twist.angular.z

    def callback_state_altitude(self,msg):
        self.lolo.altitude = msg.data

    def callback_state_pos(self,msg):
        #print("new state")
        self.lolo.lat = msg.pose.pose.position.x
        self.lolo.lon = msg.pose.pose.position.y
        self.lolo.depth = msg.pose.pose.position.z

        quaternion = ( msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.lolo.roll = euler[0]
        self.lolo.pitch = euler[1]
        self.lolo.yaw = euler[2] #unwrap!

        newMsg = EstimatedState()
        
        newMsg.lat =     self.lolo.lat                      # Latitude (WGS-84).
        newMsg.lon =     self.lolo.lon                      # Longitude (WGS-84).
        newMsg.height =  0                                  # Height (WGS-84).
        
        newMsg.phi =     self.lolo.roll                     # (roll)  #Rotation over x axis.
        newMsg.theta =   self.lolo.pitch                    # (pitch) #Rotation over y axis.
        newMsg.psi =     self.lolo.yaw                      # (yaw)   #Rotation over z axis.
        
        newMsg.u   =     self.lolo.vx                       # vxmsg.Body-Fixed xx Velocity.
        newMsg.v   =     self.lolo.vy                       # Body-Fixed yy Velocity.
        newMsg.w   =     self.lolo.vz                       # Body-Fixed zz Velocity.

        lolo_vel = np.sqrt(self.lolo.vx**2 + self.lolo.vy**2)
        newMsg.vx  =     np.cos(self.lolo.yaw)*lolo_vel     # Ground Velocity X (North).
        newMsg.vy  =     np.sin(self.lolo.yaw)*lolo_vel     # Ground Velocity Y (East).
        newMsg.vz  =     self.lolo.vz                       # Ground Velocity Z (Down).
        newMsg.p   =     self.lolo.rotX                     # Angular Velocity in x.
        newMsg.q   =     self.lolo.rotY                     # Angular Velocity in y.
        newMsg.r   =     self.lolo.rotZ                     # Angular Velocity in z.
        newMsg.depth =   self.lolo.depth                    # Depth.
        newMsg.alt =     self.lolo.altitude                 # Altitude.

        self.state_publisher.publish(newMsg)

        remoteMsg = RemoteState()
        remoteMsg.lat = newMsg.lat-0.00001
        remoteMsg.lon = newMsg.lon
        remoteMsg.psi = newMsg.psi
        remoteMsg.depth = newMsg.depth
        self.remote_state_publisher.publish(remoteMsg)

    def callback_gps(self, msg):
        #print("new GPS")
        self.gps_publisher_fix.publish(msg)
        self.gps_publisher_nav.publish(msg)

    def callback_captainstatus(self,msg):
      #print("new captain status")
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

      msg_heading = DesiredHeading() 
      msg_heading.value = msg.targetYaw
      self.targetYaw_publisher.publish(msg_heading)

      msg_pitch = DesiredPitch() 
      msg_pitch.value = msg.targetPitch
      self.targetPitch_publisher.publish(msg_pitch)

      msg_speed = DesiredSpeed() 
      msg_speed.value = msg.targetSpeed
      self.targetSpeed_publisher.publish(msg_speed)

      msg_z = DesiredZ() 
      msg_z.value = msg.targetDepth 
      msg_z.z_units = msg_z.Z_DEPTH
      self.targetDepth_publisher.publish(msg_z)

      


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
