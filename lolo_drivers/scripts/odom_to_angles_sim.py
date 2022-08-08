#!/usr/bin/env python

import rospy
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

def quaternion_multiply(q0, q1):
    # Quaternion: [x y z w]

    x0 = q0[0]
    y0 = q0[1]
    z0 = q0[2]
    w0 = q0[3]

    x1 = q1[0]
    y1 = q1[1]
    z1 = q1[2]
    w1 = q1[3]

    # Computer the product of the two quaternions, term by term
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1

    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([q0q1_x, q0q1_y, q0q1_z, q0q1_w])

    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
    return final_quaternion


def odom_callback(data):
    # Stonefish: NED 
    # WEBGUI: ENU

    quat_ned = data.pose.pose.orientation
    # NED -> ENU conversion
    quat_enu = quaternion_multiply([math.sqrt(2)/2, math.sqrt(2)/2, 0, 0], [quat_ned.x, quat_ned.y, quat_ned.z, quat_ned.w])
    rot = R.from_quat([quat_enu[0], quat_enu[1], quat_enu[2], quat_enu[3]])
    euler = rot.as_euler('xyz')

    yaw_pub = rospy.Publisher('lolo/dr/yaw', Float64, queue_size=1)
    pitch_pub = rospy.Publisher('lolo/dr/pitch', Float64, queue_size=1)
    roll_pub = rospy.Publisher('lolo/dr/roll', Float64, queue_size=1)

    roll = Float64()
    pitch = Float64()
    yaw = Float64()

    roll.data = float(euler[0])
    pitch.data = float(euler[1])
    yaw.data = float(euler[2])

    roll_pub.publish(roll)
    pitch_pub.publish(pitch)
    yaw_pub.publish(yaw)


def main():
    rospy.init_node("odom_to_angles_sim")
    rospy.Subscriber('lolo/sim/odom', Odometry, odom_callback)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    main()
