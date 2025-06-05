
# General



# ROS
import rclpy
from rclpy.node import Node

import time
from collections import deque
from rclpy.node import Node


class TopicRateMonitor:
    def __init__(self, node: Node,
                 topics_dict: dict,
                 timeout_time_sec: float=5.0,
                 window_size: int = 5,
                 report_interval: float = 1.0,
                 verbose: bool = False):
        """
        Will check that the topics are maintained at the desired rate
        Args:
            node: The ROS 2 node using this monitor.
            topics: A dict {topic_name: msg_type}
            window_size: Sliding window size for rate calculation.
            report_interval: Seconds between each rate log per topic.
        """
        #
        self.node = node
        self.topics_dict = topics_dict  # { topic_name: [message_type, desired_rate]}
        self.window_size = window_size
        self.timout_time_sec = timeout_time_sec
        self.report_interval = report_interval

        self.verbose = verbose
        # self.report_timer_rate =  # IGNORE FOR NOW

        self.timers = {}
        self.timestamps = {}
        self.timer_output = {}

        self.ready = False
        self.fault = False

        for topic, msg_info in self.topics_dict.items():
            msg_type, msg_rate = msg_info
            self._log(f"Monitoring '{topic}' at {report_interval}s interval")
            self.timestamps[topic] = deque(maxlen=window_size)
            self.timer_output[topic] = False
            node.create_subscription(msg_type, topic, self._make_callback(topic), 10)
            # TODO - for now ignoring the individual timers
            # self.timers[topic] = node.create_timer(report_interval, self._make_reporter(topic))

        # Timer for checking if topics have been received at least once
        self.report_timer = self.node.create_timer(timer_period_sec=float(self.report_interval),
                                                   callback=self.report_callback)

    def _log(self, message):
        self.node.get_logger().info(message)

    def _make_callback(self, topic_name):
        def subscriber_callback(msg):
            self.node.get_logger().info(f"Subscription callback: {topic_name}")
            self.timestamps[topic_name].append(self.node.get_clock().now())
        return subscriber_callback

    def _make_timer(self, topic_name):
        def timer_callback():
            self.node.get_logger().info(f"Timer callback: {topic_name}")
            self.timer_output[topic_name] = True
            self.timestamps[topic_name].append(self.node.get_clock().now().nanoseconds/1e9)
        return timer_callback

    # Use this if it is desired that each topic has a timer
    # For now I will just check at a given rate
    # def _make_reporter(self, topic_name):
    #     def report():
    #         topic_names = self.topics_dict.keys()
    #         for topic_name in topic_names:
    #
    #             times = self.timestamps[topic_name]
    #             if len(times) < 2:
    #                 self.node.get_logger().info(f"[{topic_name}] Waiting for data...")
    #                 return
    #
    #             intervals = [t2 - t1 for t1, t2 in zip(times, list(times)[1:])]
    #             if intervals:
    #                 avg_rate = 1.0 / (sum(intervals) / len(intervals))
    #                 self.node.get_logger().info(f"[{topic_name}] Rate: {avg_rate:.2f} Hz")
    #             else:
    #                 self.node.get_logger().info(f"[{topic_name}] Insufficient data.")
    #     return report

    def report_callback(self):
        self.determine_ready()
        self.determine_fault()

    def determine_ready(self):
        if self.ready:
            return True
        for topic_name in self.topics_dict.keys():
            if len(self.timestamps[topic_name]) == 0:
                return False

        # Set to ready
        self.ready = True
        return True

    def determine_fault(self):
        for topic_name, msg_info in self.topics_dict.items():
            msg_type, msg_rate = msg_info
            times = self.timestamps[topic_name]
            if len(times) < 2:
                if self.verbose:
                    self.node.get_logger().info(f"[{topic_name}] Waiting for data...")
                continue

            intervals = [t2 - t1 for t1, t2 in zip(times, list(times)[1:])]
            if intervals:
                avg_rate = 1.0 / (sum(intervals) / len(intervals))

                if self.verbose:
                    self.node.get_logger().info(f"[{topic_name}] Rate: {avg_rate:.2f} Hz")

                    if avg_rate < msg_rate:
                        self.fault = True
                        return True

        return False








