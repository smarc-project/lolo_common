#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def main():
    pub = rospy.Publisher('/lolo/console_in', String, queue_size=10)
    rospy.init_node('menuWriter', anonymous=True)
    while not rospy.is_shutdown():
        try:
            cmd = raw_input("Write command: ")
            if(cmd == "EXIT"): break
            #print(type(cmd))
            pub.publish(cmd+'\r\n')
        except:
            try:
                cmd = input("Write command: ")
                if(cmd == "EXIT"): break
                #print(type(cmd))
                pub.publish(cmd+'\r\n')
            except:
                pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
