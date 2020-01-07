#!/usr/bin/python

import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32
import numpy as np


def main():
    print("Starting")
    rospy.init_node('lolo_control_test', anonymous=False)

    heartbeat_publisher = rospy.Publisher('/lolo/Heartbeat', Empty, queue_size=1)

    thrusterPort = rospy.Publisher('/lolo/ctrl/thruster_port_cmd', Float32, queue_size=1)
    thrusterStrb = rospy.Publisher('/lolo/ctrl/thruster_strb_cmd', Float32, queue_size=1)
    rudderPort   = rospy.Publisher('/lolo/ctrl/rudder_port_cmd', Float32, queue_size=1)
    rudderStrb   = rospy.Publisher('/lolo/ctrl/rudder_strb_cmd', Float32, queue_size=1)
    elevator     = rospy.Publisher('/lolo/ctrl/elevator_cmd', Float32, queue_size=1)

    print("publishers initialized")

    rudderPort_request = 0.
    rudderStrb_request = 0.
    elevator_request   = 0.
    thrusterPort_request = 0.
    thrusterStrb_request = 0.

    rate = rospy.Rate(50) # 50hz
    t = 0.
    while not rospy.is_shutdown():

        heartbeat_publisher.publish()


        t+=0.01;

        rudderPort_request = 0#np.radians(30) * np.sin(t);
        rudderStrb_request = 0#np.radians(30) * np.sin(t);
        elevator_request   = 0#np.radians(30) * np.sin(t);
        thrusterPort_request = 100#-100.0 * np.sin(t);
        thrusterStrb_request = 100#100.0 * np.sin(t);

        header = Header()
        thrusterPort.publish(thrusterPort_request)
        thrusterStrb.publish(thrusterStrb_request)
        rudderPort.publish(rudderPort_request)
        rudderStrb.publish(rudderStrb_request)
        elevator.publish(elevator_request)

        rate.sleep()

if __name__ == "__main__":
    main()
