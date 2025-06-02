import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Menu_in(Node):

    def __init__(self):
        super().__init__('menu_input')
        self.publisher_ = self.create_publisher(String, '/lolo/debug/menu_in', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        cmd = input("Write command: ")
        msg = String()
        msg.data = cmd
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    m = Menu_in()
    rclpy.spin(m)
    m.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()