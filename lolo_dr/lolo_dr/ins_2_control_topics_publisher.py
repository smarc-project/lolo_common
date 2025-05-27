#!/usr/bin/python

# General
import yaml
import math

# ROS
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile
from rclpy import time
import tf_transformations

# Messages
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from ixblue_ins_msgs.msg import Ins

# SMaRC Topics
from lolo_msgs.msg import Topics as LoloTopics

try:
    from .helpers.ros_helpers import rcl_time_to_secs
except ImportError:
    from helpers.ros_helpers import rcl_time_to_secs


class Ins2Control(Node):
    """
    This node will convert output from the INS/IMU data to individual Float32 messages
    """

    def __init__(self, namespace=None):
        super().__init__("ins_2_odom", namespace=namespace)
        self._log("Starting node to convert INS/IMU data to Lolo control topics")

        # ===== Declare parameters =====
        # Default values set in declare_parameters()
        self.declare_node_parameters()

        # ===== Get parameters =====
        # Note: All parameters must be declared first! see self.declare_parameters
        # Example: self.map_frame = self.get_parameter("map_frame").value

        # Input topic names
        self.input_ins_topic = self.get_parameter("input_ins_topic").value
        self.input_imu_topic = self.get_parameter("input_imu_topic").value

        self.output_rate = self.get_parameter("output_rate").value
        self.timeout =self.get_parameter("timeout").value

        # Node behaviour parameters
        self.verbose = self.get_parameter("verbose").value

        # Data
        self.current_ins = None
        self.current_imu = None
        self.current_imu_time = None

        # Subscribers
        # IND data subscription (ins_sub):

        # TF Messages subscription (frame_sub):
        # This sub is used to determine the utm zone that we are working in.
        # Once determined we will remove this sub

        self.ins_sub = self.create_subscription(msg_type=Ins, topic=self.input_ins_topic,
                                                callback=self.ins_callback,
                                                qos_profile=QoSProfile(depth=1))

        self.imu_sub = self.create_subscription(msg_type=Imu, topic=self.input_imu_topic,
                                                callback=self.imu_callback,
                                                qos_profile=QoSProfile(depth=1))


        # Publishers
        # These are hard coded for now
        #
        # === Yaw ===
        self.ctrl_yaw_pub = self.create_publisher(msg_type=Float32,
                                                  topic=LoloTopics.CONTROL_YAW_TOPIC,
                                                  qos_profile=QoSProfile(depth=1))

        self.ctrl_yaw_rate_pub = self.create_publisher(msg_type=Float32,
                                                       topic=LoloTopics.CONTROL_YAW_RATE_TOPIC,
                                                       qos_profile=QoSProfile(depth=1))

        # === Pitch ===
        self.ctrl_pitch_pub = self.create_publisher(msg_type=Float32,
                                                    topic=LoloTopics.CONTROL_PITCH_TOPIC,
                                                    qos_profile=QoSProfile(depth=1))

        self.ctrl_pitch_rate_pub = self.create_publisher(msg_type=Float32,
                                                         topic=LoloTopics.CONTROL_PITCH_RATE_TOPIC,
                                                         qos_profile=QoSProfile(depth=1))

        # === Roll ===
        self.ctrl_roll_pub = self.create_publisher(msg_type=Float32,
                                                   topic=LoloTopics.CONTROL_ROLL_TOPIC,
                                                   qos_profile=QoSProfile(depth=1))

        self.ctrl_roll_rate_pub = self.create_publisher(msg_type=Float32,
                                                        topic=LoloTopics.CONTROL_ROLL_RATE_TOPIC,
                                                        qos_profile=QoSProfile(depth=1))

        # === Other ===
        self.ctrl_surge_rate_pub = self.create_publisher(msg_type=Float32,
                                                         topic=LoloTopics.CONTROL_SURGE_RATE_TOPIC,
                                                         qos_profile=QoSProfile(depth=1))

        self.ctrl_depth_pub = self.create_publisher(msg_type=Float32,
                                                    topic=LoloTopics.CONTROL_DEPTH_TOPIC,
                                                    qos_profile=QoSProfile(depth=1))


        # Timer for actual publishing
        self.create_timer(timer_period_sec=float(1.0/self.output_rate),
                          callback=self.publisher_callback)

    def _log(self, message):
        self.get_logger().info(message)

    # Basic node set up
    def declare_node_parameters(self):
        # Declare all the default values for parameters

        # Topic names
        self.declare_parameter("input_ins_topic", LoloTopics.INS_RAW_TOPIC)
        self.declare_parameter("input_imu_topic", "/standard/imu")

        self.declare_parameter("output_rate", 10.0)
        self.declare_parameter("timeout", 1.0)

        # Verbose output
        self.declare_parameter("verbose", False)

    # Callbacks
    def ins_callback(self, ins_msg):
        # Record message
        self.current_ins = ins_msg

    def imu_callback(self, imu_msg):
        self.current_imu = imu_msg
        self.current_imu_time = self.get_clock().now()

    def publisher_callback(self):
        # IMU
        if self.current_imu is not None and self.current_imu_time is not None:
            if self.verbose:
                self._log(f"IMU received")
        else:
            return

        last_time = self.current_imu_time.nanoseconds / 1e9
        now_time = self.get_clock().now().nanoseconds/ 1e9



        if (now_time - last_time) > self.timeout:
            if self.verbose:
                self._log(f"Imu timeout!")
            return

        if 'ned' in self.current_imu.header.frame_id:
            is_ned = True
        else:
            is_ned = False

        orientation_q = self.current_imu.orientation
        orientation_rpy = tf_transformations.euler_from_quaternion([orientation_q.x,
                                                                    orientation_q.y,
                                                                    orientation_q.z,
                                                                    orientation_q.w])

        # === Orientations ===
        # Roll
        roll_msg = Float32()
        roll_msg.data = orientation_rpy[0]
        self.ctrl_roll_pub.publish(roll_msg)

        # Pitch
        pitch_msg = Float32()
        pitch_msg.data = orientation_rpy[1]
        self.ctrl_pitch_pub.publish(pitch_msg)

        # Yaw
        yaw_msg = Float32()
        yaw_msg.data = orientation_rpy[2]
        self.ctrl_yaw_pub.publish(yaw_msg)

        # === Rates ===
        # Roll
        roll_msg.data = self.current_imu.angular_velocity.x
        self.ctrl_roll_rate_pub.publish(roll_msg)

        # pitch
        pitch_msg.data = self.current_imu.angular_velocity.y
        self.ctrl_pitch_rate_pub.publish(pitch_msg)

        # yaw
        yaw_msg.data = self.current_imu.angular_velocity.z
        self.ctrl_yaw_rate_pub.publish(yaw_msg)

        # surge
        surge_msg = Float32()
        surge_msg.data = self.current_imu.linear_acceleration.x
        self.ctrl_surge_rate_pub.publish(surge_msg)


def main(args=None, namespace=None):
    rclpy.init(args=args)
    ins_2_control_node = Ins2Control(namespace=namespace)
    try:
        rclpy.spin(ins_2_control_node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    default_namespace = "lolo"
    main(namespace=default_namespace)
