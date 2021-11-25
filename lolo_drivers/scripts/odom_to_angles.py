#!/usr/bin/env python3
import rospy
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

def odom_callback(data):
    quat = data.pose.pose.orientation
    rot = R.from_quat([quat.x, quat.y, quat.z, quat.w])
    euler = rot.as_euler('xyz')

    yaw_pub = rospy.Publisher('lolo/dr/yaw', Float64, queue_size=1)
    pitch_pub = rospy.Publisher('lolo/dr/pitch', Float64, queue_size=1)
    roll_pub = rospy.Publisher('lolo/dr/roll', Float64, queue_size=1)

    yaw = Float64()
    pitch = Float64()
    roll = Float64()

    yaw.data = float(euler[2])
    pitch.data = float(euler[1])
    roll.data = float(euler[0])

    yaw_pub.publish(yaw)
    pitch_pub.publish(pitch)
    roll_pub.publish(roll)

def main():
    rospy.init_node("odom_to_angles")
    rospy.Subscriber('lolo/dr/odom', Odometry, odom_callback)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    main()

