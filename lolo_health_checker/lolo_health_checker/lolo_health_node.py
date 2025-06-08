# General
import yaml
from dataclasses import dataclass

# ROS
import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory

# Messages
from std_msgs.msg import Int8, Float32
from lolo_msgs.msg import Pressures, Status, Temperatures

# SMaRC Topics
from lolo_msgs.msg import Topics as LoloTopics
from smarc_msgs.msg import Topics as SmarcTopics


@dataclass
class StatusReport:
    ready: bool
    fault: bool


class HealthNode(Node):
    """
    This node will check health related topics and report overall
    status to the BT at a specified rate.
    """
    # TODO - Add the ability to monitor the update rate of sensors

    def __init__(self, namespace=None):
        super().__init__("health_node", namespace=namespace)
        self._log("Starting node defined in lolo_health_node.py")

        # # Desired values
        # # === Load desired values from YAML file
        self.declare_parameter("limits_filename", "lolo_health_limits.yaml")
        self.limits_filename = self.get_parameter("limits_filename").value
        self.limits = self.read_limits()

        # TODO - for now only indicated values will be checked, should there be a default?
        self.valid_pressure_values = {
            "usbl_isb": [250, 1500],
            "thrusters_isb": [250, 1500],
            "vertical_thrusters_isb": [250, 1500],
            "prevco_isb": [250, 1500],
            # "####_isb": [0, 1],
            "battery1_isb": [250, 1500],
            "battery2_isb": [250, 1500],
        }

        self.valid_status_values = {
            "status_a": [0, 1],
            "status_b": [0, 1],
            # "rc_signal"
            "voltage": [22, 40],
            # "current"
            "captain_leak": [0],
            "esc_leak": [0],
            "prevco_leak": [0],
            "edw_leak": [0],
            "battery1_leak": [0],
            "battery2_leak": [0],

            # ISB status
            "time_status": [1],
            "trigger_status": [1],
            "actuators_status": [1],
            "thrusters_status": [1],
            "vertical_thrusters_status": [1],
            "usbl_status": [1],
            "scientist_status": [1],
            "edw_status": [1],
            "battery1_status": [1],
            "battery2_status": [1],
        }

        self.valid_temperature_values = {
            "temperature_max": [0, 1],
            "temperature_min": [0, 1],
        }

        self.valid_emergency_values = {
        # NO EMERGENCY is the only "valid" emergency value, all others are scary.
            "emergency_status": "NO EMERGENCY"
        }

        # TODO - Consider monitoring these topics
        # self.leak_topic = self.get_parameter("leak_topic").value  # Not used
        # self.battery_1_topic = self.get_parameter("battery_1_topic").value  # Not used
        # self.battery_2_topic = self.get_parameter("battery_2_topic").value  # Not used

        # ===== Data ====
        self.current_pressure = None
        self.current_pressure_time = None  # time of last received message
        self.current_status = None
        self.current_status_time = None  # time of last received message
        self.current_temperature = None
        self.current_temperature_time = None  # time of last received message
        self.current_depth = None
        self.current_depth_time = None  # time of last received message
        self.current_altitude = None
        self.current_altitude_time = None
        self.dive_start_time = None
        self.diving = False

        # ===== Status =====
        self.status = SmarcTopics.VEHICLE_HEALTH_WAITING
        # Indicates whether all the monitored topics have been received
        self.topics_status = False

        # ===== Behavior Parameters =====
        self.declare_parameter("output_rate", 1)
        self.output_rate = self.get_parameter("output_rate").value

        pressure_topic = LoloTopics.EXTENDED_INTERNAL_PRESSURE_TOPIC
        self.pressure_sub = self.create_subscription(msg_type=Pressures,
                                                     topic=pressure_topic,
                                                     callback=self.pressure_callback,
                                                     qos_profile=10)

        status_topic = LoloTopics.EXTENDED_STATUS_TOPIC
        self.status_sub = self.create_subscription(Status,
                                                   status_topic,
                                                   self.status_callback,
                                                   10)

        temperature_topic = LoloTopics.EXTENDED_INTERNAL_TEMPERATURE_TOPIC
        self.temperature_sub = self.create_subscription(msg_type=Temperatures,
                                                        topic=temperature_topic,
                                                        callback=self.temperature_callback,
                                                        qos_profile=10)

        self.depth_sub = self.create_subscription(msg_type=Float32,
                                                  topic=SmarcTopics.DEPTH_TOPIC,
                                                  callback=self.depth_callback,
                                                  qos_profile=10)

        self.altitude_sub = self.create_subscription(msg_type=Float32,
                                                  topic=SmarcTopics.ALTITUDE_TOPIC,
                                                  callback=self.altitude_callback,
                                                  qos_profile=10)

        self.check_pub = self.create_publisher(Int8,
                                               SmarcTopics.VEHICLE_HEALTH_TOPIC,
                                               10)

        # ===== Timers =====
        self.publisher_timer = self.create_timer(timer_period_sec=float(1.0 / self.output_rate),
                                                 callback=self.publisher_callback)

    def read_limits(self):
        """
        Read YAML file with Lolo's limits.
        Returns a dictionary with the values.
        """
        if not self.limits_filename:
            self.limits_filename = "lolo_health_limits.yaml"

        limits = None
        path_to_pkg = get_package_share_directory('health_checker')
        with open(path_to_pkg + "/config/" + self.limits_filename, 'r') as file:
            limits = yaml.safe_load(file)
        self.get_logger().info(f"Virtual Lolo has been configured with filename {self.limits_filename}")
        [self.get_logger().info(f"{key}:{value}") for key, value in limits.items()]

        return limits

    def _log(self, message):
        self.get_logger().info(message)

    def pressure_callback(self, msg):
        self.current_pressure = msg
        self.current_pressure_time = self.get_clock().now().nanoseconds / 1e9

    def status_callback(self, msg):
        self.current_status = msg
        self.current_status_time = self.get_clock().now().nanoseconds / 1e9

    def temperature_callback(self, msg):
        self.current_temperature = msg
        self.current_temperature_time = self.get_clock().now().nanoseconds / 1e9

    def altitude_callback(self, msg):
        self.current_altitude = msg.data
        self.current_altitude_time = self.get_clock().now().nanoseconds / 1e9

    def depth_callback(self, msg):
        self.current_depth = msg.data
        self.current_depth_time = self.get_clock().now().nanoseconds / 1e9

    def checker(self, current_msg, current_msg_valid):
        """
        return StatusReport
        """

        status = StatusReport(ready=False, fault=False)

        if current_msg is None:
            # TODO: should this return fault=True?
            return status
        else:
            status.ready = True

        for key, values in current_msg_valid.items():
            if len(values) == 1:
                if current_msg[key] != values[0]:
                    status.fault = True
                    break
            else:
                if not (values[0] <= current_msg.key <= values[1]):
                    status.fault = True
                    break

        return status

    def check_pressure(self):
        """
        return StatusReport
        """
        return self.checker(self.current_pressure, self.valid_pressure_values)

    def check_status(self):
        """
        return StatusReport
        """
        return self.checker(self.current_status, self.valid_pressure_values)

    def check_temperature(self):
        """
        return StatusReport
        """
        return self.checker(self.current_temperature, self.valid_temperature_values)

    def check_emergency(self):
        """
        Check if captain's emergency has been triggered.
        """
        status = StatusReport(ready=False, fault=False)
        if self.current_status is None:
            return status
        if self.current_status.emergency_status == "NO EMERGENCY":
            status.ready = True
            return status
        else:
            status.fault = True
        return status

    def check_depth(self):
        """
        Check if depth exceeds the maximum limit.

        return StatusReport.
        """
        status = StatusReport(ready=False, fault=False)
        if self.current_depth is None:
            return status
        status.ready = True
        if self.current_depth > self.limits["max_depth"]:
            status.fault = True
            self.get_logger().warning(f"WTF are you doing at {self.current_depth} m deep?!")
        return status

    def check_altitude(self):
        """
        Check if altitude exceeds the minimum limit.

        return StatusReport.
        """
        status = StatusReport(ready=False, fault=False)
        if self.current_altitude is None:
            return status
        status.ready = True
        if self.current_altitude < self.limits["min_altitude"]:
            status.fault = True
            self.get_logger().warning(f"Altitude of {self.current_altitude} is too scary, Aborting!")
        return status

    def check_dive_timeout(self):
        """
        Check if Lolo's been diving longer than the max limit.

        return StatusReport.
        """
        status = StatusReport(ready=False, fault=False)
        if self.diving:
            dive_time = self.current_depth_time - self.diving_start_time
            if dive_time > self.limits["max_dive_time"]:
                status.fault=True
                self.get_logger().warning(f"Total divetime of {dive_time} reached! Aborting!")
        return status

    def publisher_callback(self):
        """
        Do all the checking here
        """

        # pressure_check = self.check_pressure()
        # status_check = self.check_status()
        # temperature_check = self.check_temperature()
        depth_check = self.check_depth()
        altitude_check = self.check_altitude()
        dive_check = self.check_dive_timeout()
        emergency_check = self.check_emergency()

        # ready_checks = [pressure_check.ready, status_check.ready,
                        # temperature_check.ready]
        ready_checks = [emergency_check.ready, depth_check.ready,
                        altitude_check.ready]

        # fault_checks = [pressure_check.fault, status_check.fault,
                        # temperature_check.fault, emergency_check.fault,
                        # depth_check.fault, altitude_check.fault,
                        # dive_check.fault]
        fault_checks = [emergency_check.fault, depth_check.fault,
                        altitude_check.fault, dive_check.fault]

        if True in fault_checks:
            msg = Int8()
            msg.data = SmarcTopics.VEHICLE_HEALTH_ERROR
            self.check_pub.publish(msg)
        elif all(ready_checks):
            msg = Int8()
            msg.data = SmarcTopics.VEHICLE_HEALTH_READY
            self.check_pub.publish(msg)
        else:
            msg = Int8()
            msg.data = SmarcTopics.VEHICLE_HEALTH_WAITING
            self.check_pub.publish(msg)

def main(args=None, namespace=None):
    rclpy.init(args=args)
    node = HealthNode(namespace=namespace)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
        rclpy.shutdown()


if __name__ == "__main__":
    default_namespace = "lolo"
    main(namespace=default_namespace)
