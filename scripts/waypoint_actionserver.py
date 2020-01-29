#!/usr/bin/python

from __future__ import division, print_function

import scipy.special
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PointStamped, Pose, Point
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseResult, MoveBaseAction
import math
import actionlib
import rospy
from smarc_msgs.msg import Float32Stamped

class WaypointServer(object):

    #current position of lolo
    _current_pose = Pose()
    goal_tolerance = 10

    # create messages that are used to publish feedback/result
    _feedback = MoveBaseFeedback()
    _result = MoveBaseResult()

    def calculatedistance(self, lat1, lon1, lat2, lon2):
        # Using Haversine formula
        R = 6378127; # metres
        phi1 = lat1; # rad
        phi2 = lat2; #rad
        dphi = (lat2-lat1); #rad
        dlambda = (lon2-lon1); #rad

        a = math.sin(dphi/2) * math.sin(dphi/2) + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2) * math.sin(dlambda/2);
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a));

        return (R * c);

    def pose_callback(self, msg):
        #print("Lolo position received")
        self._current_pose = msg.pose

    def callback(self, pose_msg):
        if len(pose_msg.header.frame_id) == 0:
            self.nav_goal = None
            return
        self.nav_goal = pose_msg.pose

    def execute_cb(self, goal):
        # helper variables
        success = True
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
                wp = Point()
                wp.x = self.nav_goal.position.x
                wp.y = self.nav_goal.position.y
                self.wp_publisher.publish(wp)
                self.speed_publisher.publish(Header(),targetSpeed)

                self._feedback.base_position = PoseStamped()
                self._feedback.base_position.pose = self._current_pose
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
        print("Nav goal is not None!")

        #Check distance to goal
        lolo_lat = self._current_pose.position.x
        lolo_lon = self._current_pose.position.y
        goal_lat = self.nav_goal.position.x
        goal_lon = self.nav_goal.position.y

        #print("Checking if nav goal is reached!")
        dist = self.calculatedistance(lolo_lat, lolo_lon, goal_lat, goal_lon)
        print("Distance to goal: " + str(dist) + " m")
        if dist < self.goal_tolerance:
            rospy.loginfo("Reached goal!")
            self.nav_goal = None
        #else:
        #    print("Did not reach nav goal!")

    def __init__(self, name):


        self._action_name = name

        self.base_frame = rospy.get_param('~base_frame', "you/forgot/base_frame/param")

        self.nav_goal = None

        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback)
        rospy.Subscriber("/lolo/lolo//lolo/core/state/position", PoseWithCovarianceStamped, self.pose_callback)

        self.wp_publisher = rospy.Publisher("/lolo/core/waypoint_cmd", Point, queue_size=1)
        self.speed_publisher = rospy.Publisher("/lolo/core/speed_cmd", Float32Stamped, queue_size=1)

        rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        self._as = actionlib.SimpleActionServer(self._action_name, MoveBaseAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Announced action server with name: %s", self._action_name)

        r = rospy.Rate(10) # 10hz
        counter = 0
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':

    rospy.init_node('gotoWP_actionserver')
    server = WaypointServer(rospy.get_name())
