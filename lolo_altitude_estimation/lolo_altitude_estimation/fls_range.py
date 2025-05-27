import rclpy
import rclpy.time
import rclpy.duration
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32

class Fls_range_node(Node):

    def __init__(self):
        super().__init__('fls_range')

        #maximum time difference allowed between two consecutive estimates
        self.max_time_diff = 3
        #maximum range difference allowed between two consecutuve estimates
        self.max_range_diff = 2


        self.estimator1_range = 0
        self.estimator1_time_seconds, _ = self.get_clock().now().seconds_nanoseconds()
        self.estimator1_valid = False

        self.estimator2_range = 0
        self.estimator2_time_seconds, _ = self.get_clock().now().seconds_nanoseconds()
        self.estimator2_valid = False

        self.valid_counter = 0

        self.estimator1_sub = self.create_subscription(
            Float32,
            '/altitude1',
            self.estimator1_callback,
            1)

        self.estimator2_sub = self.create_subscription(
            Float32,
            '/altitude2',
            self.estimator2_callback,
            1)

        self.output_pub = self.create_publisher(Float32, "fls_range", 2)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def estimator1_callback(self, msg):
        self.get_logger().info('Estimator1: "%s"' % msg.data)
         
        #Compase this estimate with the last to decide if it should be trusted or not
        time_now, _ = self.get_clock().now().seconds_nanoseconds()
        range_diff = abs(msg.data - self.estimator1_range)
        time_diff = time_now - self.estimator1_time_seconds
        self.estimator1_range = msg.data
        self.estimator1_time_seconds = time_now
        self.estimator1_valid = True if range_diff < self.max_range_diff and time_diff < self.max_time_diff else False
    
    def estimator2_callback(self, msg):
        self.get_logger().info('Estimator2: "%s"' % msg.data)

        #Compase this estimate with the last to decide if it should be trusted or not
        time_now, _ = self.get_clock().now().seconds_nanoseconds()
        range_diff = abs(msg.data - self.estimator2_range)
        time_diff = time_now - self.estimator2_time_seconds
        self.estimator2_range = msg.data
        self.estimator2_time_seconds = time_now
        self.estimator2_valid = True if range_diff < self.max_range_diff and time_diff < self.max_time_diff else False

    def timer_callback(self):
        self.get_logger().info("Timer callback")
        #Check if both estimators have been updated
        time_now, _ = self.get_clock().now().seconds_nanoseconds()
        estimator1_time_ok = (time_now - self.estimator1_time_seconds) < self.max_range_diff
        estimator2_time_ok = (time_now - self.estimator2_time_seconds) < self.max_range_diff

        self.get_logger().info("Estimator1: " + str(estimator1_time_ok) + ", Estimator2: " + str(estimator2_time_ok))

        if(self.estimator1_valid and self.estimator2_valid and abs(self.estimator1_range - self.estimator2_range) < 5 and estimator1_time_ok and estimator2_time_ok):
            self.valid_counter+=1
            self.get_logger().info("Both estimators valid")
        else:
            self.valid_counter = 0

        if(self.valid_counter > 2):
            range_msg = Float32()
            range_msg.data = 0.5*(self.estimator1_range + self.estimator2_range)
            self.output_pub.publish(range_msg)
            


def main(args=None):
    rclpy.init(args=args)

    fls_range_node = Fls_range_node()
    executor = MultiThreadedExecutor()
    executor.add_node(fls_range_node)
    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fls_range_self.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()