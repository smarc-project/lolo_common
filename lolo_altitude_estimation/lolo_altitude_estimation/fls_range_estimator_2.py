import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_msgs.msg import Float32

# Calculates a range estimate from the watercolumn data from Norbit WBMS
class Range_estimator(Node):

    def __init__(self):
        super().__init__('fls_range_estimator_2')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.watercolumn_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Float32, "altitude2", 2)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def watercolumn_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    #for testing
    def timer_callback(self):
        self.get_logger().info("Published altitude")
        msg = Float32()
        msg.data = 23.3
        self.publisher.publish(msg)




def main(args=None):
    rclpy.init(args=args)

    estimator = Range_estimator()
    executor = MultiThreadedExecutor()
    executor.add_node(estimator)
    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()