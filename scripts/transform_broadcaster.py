#!/usr/bin/python

#!/usr/bin/env python  
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import numpy as np
import tf
import math
from geodesy.utm import fromLatLong, UTMPoint


class TF_Broadcaster:
    def __init__(self):
        self.br_ned_utm = tf.TransformBroadcaster()
        self.br_enu_utm = tf.TransformBroadcaster()
        self.br_world = tf.TransformBroadcaster()
        
        self.state_subscriber = rospy.Subscriber("/lolo/core/state/position", PoseWithCovarianceStamped, self.callback_state)

    def callback_state(self,msg):
        print("new state")

        #Send transform World
        self.br_world.sendTransform((0,0,0), (0,0,0,1), rospy.Time.now(), "world_wgs84", "world")
        
        #Compute UTM position
        pos = fromLatLong(np.degrees(msg.pose.pose.position.x), np.degrees(msg.pose.pose.position.y))
        
        # get the grid-zone
        gz, band = pos.gridZone()
        #pos.northing
        #pos.easting

        #rotation from (xyz,xyzw)

        #self.br_ned_utm.sendTransform((pos.northing - msg.pose.pose.position.x, pos.easting  - msg.pose.pose.position.y , msg.pose.pose.position.z), msg.pose.pose.orientation,
        self.br_ned_utm.sendTransform((5 - msg.pose.pose.position.x, pos.easting  - 5 , msg.pose.pose.position.z), 
                        (msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w),
                        rospy.Time.now(),
                        "local_ned",
                        "world")
        
        self.br_enu_utm.sendTransform((pos.easting - msg.pose.pose.position.y, pos.northing  - msg.pose.pose.position.x , msg.pose.pose.position.z),
                        (msg.pose.pose.orientation.w,
                         msg.pose.pose.orientation.x,
                         msg.pose.pose.orientation.y,
                         msg.pose.pose.orientation.z),
                         rospy.Time.now(),
                        "world_enu_utm",
                        "world")

        #newMsg = Pose()
        #newMsg.position.x = msg.pose.pose.position.x
        #newMsg.position.y = msg.pose.pose.position.y
        #newMsg.position.z = msg.pose.pose.position.z
        #newMsg.orientation = msg.pose.pose.orientation
        #self.state_publisher.publish(newMsg)


def main():
    print("Starting")
    rospy.init_node('transform_broadcaster', anonymous=False)

    br = TF_Broadcaster();
    rospy.spin()

if __name__ == "__main__":
    main()
