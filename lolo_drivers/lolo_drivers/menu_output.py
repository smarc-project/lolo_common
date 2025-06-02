import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Menu_out(Node):

    def __init__(self):
        super().__init__('menu_output')

        self.declare_parameter("menu_out_topic", '/lolo/debug/menu_out')
        self.menu_out_topic = self.get_parameter("menu_out_topic").get_parameter_value().string_value

        self.subscription = self.create_subscription(
            String,
            self.menu_out_topic,
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        print(msg.data, end='')


def main(args=None):
    rclpy.init(args=args)

    m = Menu_out()
    rclpy.spin(m)
    m.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()