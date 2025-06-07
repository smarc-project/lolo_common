import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nortek_dvl333_msgs.msg import BottomTrack
from lolo_msgs.msg import Topics as LoloTopics
from smarc_msgs.msg import Topics as smarcTopics
from std_msgs.msg import Float32

class Altitude_node(Node):

    def __init__(self):
        super().__init__('altitude_node')

        #DVL
        self.dvl_altitude = None

        self.dvl_subscription = self.create_subscription(
            BottomTrack,
            LoloTopics.DVL_TOPIC,
            self.dvl_callback,
            10)

        self.publisher = self.create_publisher(Float32, smarcTopics.ALTITUDE_TOPIC, 2)


        self.timer = self.create_timer(0.5, self.timer_callback)

        

    def dvl_callback(self, msg):
        #Check if dvl data is valid
        beam_sum = 0
        valid_beams = 0

        beamlist = [msg.dist_beam0,
                    msg.dist_beam1,
                    msg.dist_beam2,
                    msg.dist_beam3]
        
        for beamRange in beamlist:
            if beamRange > 0.3 and beamRange < 400:
                beam_sum+=beamRange
                valid_beams+=1

        if(valid_beams > 0):
            self.dvl_altitude = beam_sum / valid_beams
        else:
            self.dvl_altitude = None



    def timer_callback(self):
        if(self.dvl_altitude is not None):
            self.get_logger().info("DVL data received")
            msg = Float32()
            msg.data = self.dvl_altitude
            self.publisher.publish(msg)
            


def main(args=None):
    rclpy.init(args=args)

    node = Altitude_node()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fls_range_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()