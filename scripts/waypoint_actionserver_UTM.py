#!/usr/bin/python

from __future__ import division, print_function

import scipy.special
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PointStamped, Pose, Point
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseResult, MoveBaseAction
import math
import actionlib
import rospy
from smarc_msgs.msg import Float32Stamped, UTMposeStamped, UTMpoint

class WaypointServer(object):

    #goal tolerance. This should probably be a parameter..
    GOAL_TOLERANCE = 10

    #current position of lolo
    _current_position_UTM = UTMpoint()

    # create messages that are used to publish feedback/result
    _feedback = MoveBaseFeedback()
    _result = MoveBaseResult()

    ############# Get current position of Lolo #############
    def pose_callback(self, msg):
        self._current_position_UTM = msg.pose.position

    ################# Actionserver stuff ###################
    def execute_cb(self, goal):
        # helper variables
        success = True
        print("Execute CB goal: \n" + str(goal))
        #current position UTM
        self.nav_goal = goal.target_pose.pose

        r = rospy.Rate(10.) # 10hz
        counter = 0
        while not rospy.is_shutdown() and self.nav_goal is not None:
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                self.nav_goal = None
                break

            if counter % 100 == 0:
                print("Publish stuff to LOLO")
                targetSpeed = 1
                wp = UTMpoint()
                wp.easting = self.nav_goal.position.x
                wp.northing = self.nav_goal.position.y
                wp.band = self._current_position_UTM.band
                wp.zone = self._current_position_UTM.zone

                self.wp_publisher.publish(wp)
                self.speed_publisher.publish(Header(),targetSpeed)

                self._feedback.base_position = PoseStamped()
                self._feedback.base_position.pose.position.x = self._current_position_UTM.easting
                self._feedback.base_position.pose.position.y = self._current_position_UTM.northing
                self._feedback.base_position.pose.position.z = self._current_position_UTM.depth

                self._feedback.base_position.header.stamp = rospy.get_rostime()
                self._as.publish_feedback(self._feedback)
            counter += 1
            r.sleep()

        # publish vel=0 to stop
        targetSpeed = 0
        #self.wp_publisher
        self.speed_publisher.publish(Header(),targetSpeed)


        if success:
            #self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


    def timer_callback(self, event):
        #print("Timer callback")
        if self.nav_goal is None:
            #print("Nav goal is None!")
            return
        
        dx = self.nav_goal.position.x - self._current_position_UTM.easting
        dy = self.nav_goal.position.y - self._current_position_UTM.northing
        dist = math.sqrt(dx*dx + dy*dy)
        print("Distance to goal: " + str(dist) + " m")
        if dist < self.GOAL_TOLERANCE:
            rospy.loginfo("Reached goal!")
            self.nav_goal = None

    def __init__(self, name):


        self._action_name = name

        self.base_frame = rospy.get_param('~base_frame', "you/forgot/base_frame/param")

        self.nav_goal = None

        #Position of lolo
        rospy.Subscriber("/lolo/core/state/position_UTM", UTMposeStamped, self.pose_callback)

        #Lolo setpoints
        self.wp_publisher = rospy.Publisher("/lolo/core/UTMwaypoint_cmd", UTMpoint, queue_size=1)
        self.speed_publisher = rospy.Publisher("/lolo/core/speed_cmd", Float32Stamped, queue_size=1)
        #self.depth_publisher = rospy.Publisher("/lolo/core/depth_cmd", Float32Stamped, queue_size=1)
        #self.altitude_publisher = rospy.Publisher("/lolo/core/depth_cmd", Float32Stamped, queue_size=1)

        #Timer
        rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        #Action server stuff
        self._as = actionlib.SimpleActionServer(self._action_name, MoveBaseAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Announced action server with name: %s", self._action_name)

        r = rospy.Rate(10) # 10hz
        counter = 0
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':

    rospy.init_node('gotoWP_actionserver_UTM')
    server = WaypointServer(rospy.get_name())
