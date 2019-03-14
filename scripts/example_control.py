#!/usr/bin/python

import rospy
from smarc_msgs.msg import RudderAngle
from smarc_msgs.msg import ThrusterRPM
from std_msgs.msg import Header
import numpy as np


def main():
    print("Starting")
    rospy.init_node('loloTest', anonymous=True)

    thrusterPort = rospy.Publisher('/lolo_auv_1/thrusters/0/input', ThrusterRPM, queue_size=10)
    thrusterStrb = rospy.Publisher('/lolo_auv_1/thrusters/1/input', ThrusterRPM, queue_size=10)
    rudderPort   = rospy.Publisher('/lolo_auv_1/fins/0/input', RudderAngle, queue_size=10)
    rudderStrb   = rospy.Publisher('/lolo_auv_1/fins/1/input', RudderAngle, queue_size=10)
    elevator     = rospy.Publisher('/lolo_auv_1/back_fins/0/input', RudderAngle, queue_size=10)

    print("publishers initialized")

    rudderPort_request = 0.
    rudderStrb_request = 0.
    elevator_request   = 0.
    thrusterPort_request = 0.
    thrusterStrb_request = 0.

    rate = rospy.Rate(10) # 10hz
    t = 0.
    while not rospy.is_shutdown():

        t+=0.01;

        rudderPort_request = np.radians(30) * np.sin(t);
        rudderStrb_request = np.radians(30) * np.sin(t);
        elevator_request   = np.radians(30) * np.sin(t);
        thrusterPort_request = 100.0 * np.sin(t);
        thrusterStrb_request = 100.0 * np.sin(t);

        header = Header()
        thrusterPort.publish(header, thrusterPort_request)
        thrusterStrb.publish(header, thrusterStrb_request)
        rudderPort.publish(header, rudderPort_request)
        rudderStrb.publish(header, rudderStrb_request)
        elevator.publish(header, elevator_request)

        rate.sleep()

if __name__ == "__main__":
    main()
