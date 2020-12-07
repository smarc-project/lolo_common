#! /usr/bin/env python

import rospy

import actionlib
import math
import smarc_msgs.msg
from std_msgs.msg import Float64
import tf
from geometry_msgs.msg import Point

class GotoWaypointAction(object):
    # create messages that are used to publish feedback/result
    _feedback = smarc_msgs.msg.GotoWaypointFeedback()
    _result = smarc_msgs.msg.GotoWaypointResult()

    def __init__(self, name, _tf_listener):
        self.tf_listerner = _tf_listener
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, smarc_msgs.msg.GotoWaypointAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        self.waypoint_pub = rospy.Publisher("lolo/ctrl/waypoint_setpoint_utm", Point, queue_size=1)
        self.rpm_pub = rospy.Publisher("lolo/ctrl/rpm_setpoint", Float64, queue_size=1)
        self.depth_pub = rospy.Publisher("lolo/ctrl/depth_setpoint", Float64, queue_size=1)
        #TODO:
        # speed
        # altude
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # publish info to the console for the user
        print("Lolo waypoint actionserver started")
        
        # start executing the action
        while not rospy.is_shutdown():
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            try:
                (trans,rot) = listener.lookupTransform('utm', '/lolo/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("tranform error")
                self._as.set_preempted()
                success = False
                break;
                pass

            #Calculate the distance to target
            dx = goal.waypoint_pose.pose.position.x - trans[0]
            dy = goal.waypoint_pose.pose.position.y - trans[1]
            dist = math.sqrt(dx*dx + dy*dy)
            
            # publish feedback
            self._feedback.ETA = rospy.get_rostime() + rospy.Duration((dist / 0.6)) #TOTO change 0.6 for actual speed
            self._as.publish_feedback(self._feedback)
            
            #Check if distance to goal < tolerance
            if(dist < goal.goal_tolerance):
                print("Goal reached")
                break; #Goal reached

            
            print("Send setpoints to lolo")
            #send setpoints to lolo
            target = goal.waypoint_pose.pose.position
            self.waypoint_pub.publish(target)
            #TODO change to altitude depending on z_control_mode
            self.depth_pub.publish(goal.travel_depth)
            self.rpm_pub.publish(goal.travel_rpm)
            
        
        print("Send 0 rpm to lolo")
        #self.waypoint_pub.publish(target)
        self.depth_pub.publish(-30)
        self.rpm_pub.publish(0)
          
        if success:
            #self._result.reached_waypoint = True
            rospy.loginfo('%s: Waypoint reached' % self._action_name)
            self._as.set_succeeded(self._result)

        
if __name__ == '__main__':
    rospy.init_node('lolo_waypoint_action')
    listener = tf.TransformListener()
    server = GotoWaypointAction(rospy.get_name(), listener)
    rospy.spin()
