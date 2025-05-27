#!/usr/bin/python

# General
import math

# ROS
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from rclpy import time
import tf_transformations

# Messages (Standard)
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu

# Messages (Specific)
from ixblue_ins_msgs.msg import Ins

# SMaRC Topics
from lolo_msgs.msg import Topics as LoloTopics

try:
    from .helpers.ros_helpers import rcl_time_to_secs
    from .helpers.spatial_helpers import heading_to_yaw
except ImportError:
    from helpers.ros_helpers import rcl_time_to_secs
    from helpers.spatial_helpers import heading_to_yaw


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
        # Input topic names
        self.input_ins_topic = self.get_parameter("input_ins_topic").value
        self.input_imu_topic = self.get_parameter("input_imu_topic").value

        # Node parameters
        self.convert_to_yaw = self.get_parameter("convert_to_yaw").value
        self.output_degrees = self.get_parameter("output_degrees").value

        # TODO - Remove timer based publishing
        self.output_rate = self.get_parameter("output_rate").value
        self.timeout = self.get_parameter("timeout").value

        self.verbose = self.get_parameter("verbose").value

        # Data
        self.current_ins = None
        self.current_ins_time = None
        self.current_imu = None
        self.current_imu_time = None

        # Subscribers
        # IND data subscription (ins_sub):
        # IMU dat subscription (imu_sub):

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

        # Timer for actual time_out timer
        # self.create_timer(timer_period_sec=float(1.0 / self.output_rate),
        #                   callback=self.publisher_callback)

    def _log(self, message):
        self.get_logger().info(message)

    # Basic node set up
    def declare_node_parameters(self):
        # Declare all the default values for parameters

        # Topic names
        self.declare_parameter("input_ins_topic", LoloTopics.INS_RAW_TOPIC)
        self.declare_parameter("input_imu_topic", "/standard/imu")

        # Conversion parameters
        self.declare_parameter("convert_to_yaw", True)
        self.declare_parameter("output_degrees", True)

        # Behavior parameters
        # if output rate is set to 0, messages will be published and the relevant topics are received.
        self.declare_parameter("output_rate", 0.0)
        self.declare_parameter("timeout", 1.0)

        # Verbose output
        self.declare_parameter("verbose", False)

    # Callbacks
    def ins_callback(self, ins_msg):
        """
        The callback from the INS ROS topic (ixblue_ins_msgs.msg.Ins) is used to publishing the following:
        - yaw
        - pitch
        - roll
        - altitude
        """
        # Record message
        self.current_ins = ins_msg
        self.current_ins_time = self.get_clock().now()

        # === Orientations ===
        # Roll
        roll_msg = Float32()
        roll_msg.data = ins_msg.roll
        self.ctrl_roll_pub.publish(roll_msg)

        # Pitch
        pitch_msg = Float32()
        pitch_msg.data = ins_msg.pitch
        self.ctrl_pitch_pub.publish(pitch_msg)

        # Yaw
        # Compute yaw from heading
        if self.convert_to_yaw:
            value_rad = heading_to_yaw(ins_msg.heading)
        else:
            value_rad = math.radians(ins_msg.heading)

        # Convert to degrees
        if self.output_degrees:
            value = math.degrees(value_rad)
        else:
            value = value_rad

        yaw_msg = Float32()
        yaw_msg.data = value
        self.ctrl_yaw_pub.publish(yaw_msg)

        # Check that the INS is set to use the correct alltitude reference
        # 0: geoid -> equivalent to mean sea level <-- Correct for our purposes
        # 1: ellipsoid -> WGS84
        if ins_msg.altitude_ref == 0:
            altitude_msg = Float32()
            altitude_msg.data = ins_msg.altitude
            self.ctrl_depth_pub.publish(altitude_msg)

    def imu_callback(self, imu_msg):
        self.current_imu = imu_msg
        self.current_imu_time = self.get_clock().now()

        # === Rates ===
        # Roll
        roll_rate_msg = Float32()
        roll_rate_msg.data = self.current_imu.angular_velocity.x
        self.ctrl_roll_rate_pub.publish(roll_rate_msg)

        # pitch
        pitch_rate_msg = Float32()
        pitch_rate_msg.data = self.current_imu.angular_velocity.y
        self.ctrl_pitch_rate_pub.publish(pitch_rate_msg)

        # yaw
        yaw_rate_msg = Float32()
        yaw_rate_msg.data = self.current_imu.angular_velocity.z
        self.ctrl_yaw_rate_pub.publish(yaw_rate_msg)

        # surge
        surge_msg = Float32()
        surge_msg.data = self.current_imu.linear_acceleration.x
        self.ctrl_surge_rate_pub.publish(surge_msg)

    def publisher_callback(self):
        if not self.verbose:
            return

        now_time = self.get_clock().now().nanoseconds / 1e9

        ins_last_time = self.current_ins_time.nanoseconds / 1e9
        imu_last_time = self.current_imu_time.nanoseconds / 1e9

        if (now_time - imu_last_time) > self.timeout:
            self._log(f"IMU timeout!")

        if (now_time - ins_last_time) > self.timeout:
            self._log(f"INS timeout")


def main(args=None, namespace=None):
    rclpy.init(args=args)
    node = Ins2Control(namespace=namespace)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        # rclpy.shutdown()


if __name__ == "__main__":
    default_namespace = "lolo"
    main(namespace=default_namespace)
