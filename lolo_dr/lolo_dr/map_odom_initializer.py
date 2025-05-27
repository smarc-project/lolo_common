# General python imports
import utm

# ROS2 specific imports
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

# ROS2 messages
from geometry_msgs.msg import TransformStamped
from ixblue_ins_msgs.msg import Ins

# ROS2 Topics
from lolo_msgs.msg import Topics as loloTopics

class MapOdomInitializer(Node):
    """
    Initializes the TF tree and use the initial GPS position from the INS to define the "odom" frame.

    Conventions used:
    "utm_{zone}_{band}" -> "utm" -> "odom"

    """
    def __init__(self):
        super().__init__('map_odom_initializer')
        self._log(f'map -> odom initialization')
        self._log(f'Waiting for initial Lat/Lon coordinates')

        self.declare_parameter("verbose", False)
        self.verbose = self.get_parameter("verbose").value

        self.subscription = self.create_subscription(
            msg_type=Ins,
            topic=loloTopics.INS_RAW_TOPIC,
            callback=self.ins_callback,
            qos_profile=10
        )

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.origin_set = False

    def ins_callback(self, msg: Ins):
        if self.origin_set:
            return

        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude

        # Use the first GPS fix as map origin
        easting, northing, zone, letter = utm.from_latlon(latitude, longitude)

        # sets the transform so that the `odom` frame origin (0,0) is placed at the lat/lon-based UTM position in `map`.

        # Time used ofr stamping the TF messages
        now = self.get_clock().now().to_msg()

        # Define utm root (includes zone and band) to utm transform
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = f'utm_{zone}_{letter}'
        tf_msg.child_frame_id = 'utm'
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.w = 1.0  # Identity rotation

        self.static_broadcaster.sendTransform(tf_msg)

        # Define utm to odom transform, based on initial lat/lon from INS
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = f'utm'
        tf_msg.child_frame_id = 'odom'
        tf_msg.transform.translation.x = easting
        tf_msg.transform.translation.y = northing
        tf_msg.transform.translation.z = altitude
        tf_msg.transform.rotation.w = 1.0  # Identity rotation

        self.static_broadcaster.sendTransform(tf_msg)
        self.origin_set = True

        self._log(f"Set static transform map → odom at UTM ({easting:.2f}, {northing:.2f})")
        self._log("Shutting down node after publishing transform.")

        # Optional: exit after sending the transform
        rclpy.shutdown()

    def _log(self, message):
        self.get_logger().info(message)

def main(args=None):
    rclpy.init(args=args)
    node = MapOdomInitializer()
    rclpy.spin(node)
