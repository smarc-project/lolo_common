import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from lolo_msgs.msg import Status
from lolo_msgs.msg import Pressures
from lolo_msgs.msg import Temperatures

import paho.mqtt.client as mqtt

class Translator_node(Node):

    def __init__(self, mqtt_node):
        super().__init__('translator_node')

        # Mqtt
        self.mqtt = mqtt_node


        # Ros2 subscribers
        self.subscription_menu_out = self.create_subscription(String,'/lolo/debug/menu_out',self.callback_menu_out,2)
        self.subscription_internal_pressure = self.create_subscription(Pressures,'/lolo/extended/internal_pressure',self.callback_internal_pressure,2)
        self.subscription_internal_temperature = self.create_subscription(Temperatures,'/lolo/extended/internal_temperature',self.callback_internal_temperature,2)
        self.subscription_status = self.create_subscription(Status,'/lolo/extended/status',self.callback_status,2)



    def callback_menu_out(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        try:
            self.mqtt.publish("/menu_out", msg.data)
        except Exception as e:
            print(e)

    def callback_internal_pressure(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        try:
            self.mqtt.publish("/internal_pressure", msg.data)
        except Exception as e:
            print(e)

    def callback_internal_temperature(self, msg):
        self.get_logger().info('I heard: "%s"' % str(msg))
        try:
            self.mqtt.publish("/internal_temperature", str(msg))
        except Exception as e:
            print(e)
    
    def callback_internal_pressure(self, msg):
        self.get_logger().info('I heard: "%s"' % str(msg))
        try:
            self.mqtt.publish("/internal_pressure", str(msg))
        except Exception as e:
            print(e)

    def callback_status(self, msg):
        self.get_logger().info('I heard: "%s"' % str(msg))
        try:
            self.mqtt.publish("/status", str(msg))
        except Exception as e:
            print(e)

        # The callback for when the client receives a CONNACK response from the server.
    def mqtt_on_connect(self,client, userdata, flags, reason_code, properties):
        print(f"Connected with result code {reason_code}")
        #TODO 

    # The callback for when a PUBLISH message is received from the server.
    def mqtt_on_message(self,client, userdata, msg):
        pass


def main(args=None):
    rclpy.init(args=args)

    mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    translator = Translator_node(mqttc)
    mqttc.on_connect = translator.mqtt_on_connect
    mqttc.on_message = translator.mqtt_on_message
    mqttc.connect("192.168.1.100", 1883, 60)
    mqttc.loop_start()

    executor = MultiThreadedExecutor()
    executor.add_node(translator)
    executor.spin()
    #rclpy.spin(translator)

    translator.destroy_node()
    mqttc.loop_stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
