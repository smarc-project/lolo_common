#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib
import smarc_msgs.msg

def client():
    client = actionlib.SimpleActionClient('lolo_waypoint_action', smarc_msgs.msg.GotoWaypointAction)

    print("Waiting for server")
    client.wait_for_server()

    print("Create goal")
    goal = smarc_msgs.msg.GotoWaypointGoal()
    goal.travel_rpm = 200
    goal.travel_rpm = 200
    
    #lolo pos
    x = 347200
    y = 6574100

    goal.waypoint_pose.pose.position.x = x
    goal.waypoint_pose.pose.position.y = y

    print("Send goal")
    client.send_goal(goal)

    print("Wait for result")
    client.wait_for_result()

    print("Done")
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('actionclient_tester')
        result = client()
        print(result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")