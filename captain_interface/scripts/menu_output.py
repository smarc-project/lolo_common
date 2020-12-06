#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    msg = str(data.data)
    #print(msg,end='')
    sys.stdout.write(msg)


def main():
    rospy.init_node('menuListener', anonymous=True)

    rospy.Subscriber('/lolo/console_out', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
