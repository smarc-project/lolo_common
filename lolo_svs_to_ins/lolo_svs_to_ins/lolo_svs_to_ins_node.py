import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from svs_interfaces.msg import SVS
import socket
import time


class SVS_sender(Node):

    def __init__(self):
        super().__init__('svs_to_ins')

        self.declare_parameter("endpoint_ip", '127.0.0.1')
        self.endpoint_ip = self.get_parameter("endpoint_ip").get_parameter_value().string_value

        self.declare_parameter("endpoint_port", 8888)
        self.endpoint_port = int(self.get_parameter("endpoint_port").value)

        self.declare_parameter("svs_topic", '/lolo/sensors/svs')
        self.svs_topic = self.get_parameter("svs_topic").get_parameter_value().string_value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        
        self.subscription = self.create_subscription(SVS,self.svs_topic, self.transmit_callback, 10)


        #Send data
    def transmit_callback(self,msg):
        #self.get_logger().info(f"received SVS message: {msg}")
        svs_data = ""+str(msg.svs) + "\r\n"
        self.sock.sendto(svs_data.encode(), (self.endpoint_ip, self.endpoint_port))



def main(args=None):
    rclpy.init(args=args)

    svs_sender = SVS_sender()

    #rclpy.spin(svs_sender)
    #svs_sender.destroy_node()

    executor = MultiThreadedExecutor()
    executor.add_node(svs_sender)
    executor.spin()

    udp_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()