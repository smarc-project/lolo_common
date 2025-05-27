import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from lolo_msgs.msg import Status
from lolo_msgs.msg import Topics
from diagnostic_msgs.msg import KeyValue

class LoloParams(Node):

    class LoloParameter:
        def __init__(self, _name, _default_value):
            self.name = _name
            self.value = _default_value

    # Store latest status message locally
    def status_callback(self, msg):
        self.lolo_status_msg = msg


    # Record change of rosparm in a local variable
    def parameter_callback(self, params):#edw_armed
        #edw_timer_armed

        for param in params:
            for lolo_param in self.lolo_parameters:
                if(param.name == lolo_param.name):
                    lolo_param.value = param.value

        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__('lolo_params_node')

        #Feedback message
        self.lolo_status_msg = Status()
        
        #Parameters
        self.lolo_parameters = [self.LoloParameter('control_source', 'EXTERNAL_CONTROL'), #NONE, MANUAL_CONTROL, ONBOARD_CONTROL, EXTERNAL_CONTROL, EMERGENCY_CONTROL
                                self.LoloParameter('control_mode', 'STANDBY'), #STANDBY, DISARMED, ARMED
                                self.LoloParameter('thrusters_enabled', 0), #1 (True) / 0 (False)
                                self.LoloParameter('vertical_thrusters_enabled', 0)] #1 (True)/ 0 (False)

        #Declare parameters
        for param in self.lolo_parameters:
            self.declare_parameter(param.name, param.value)

        #Local variables with parameter value
        for param in self.lolo_parameters:
            param.value = self.get_parameter(param.name).value

        #Print params for debugging
        print("Parameters:")
        for param in self.lolo_parameters:
            print(param.name + " : " + str(param.value))

        # Set callback for changed parameters
        self.add_on_set_parameters_callback(self.parameter_callback)

        #Create publishers and subscribers
        self.status_subscription = self.create_subscription(Status,Topics.EXTENDED_STATUS_TOPIC, self.status_callback,10)
        self.status_subscription  # prevent unused variable warning
        self.settings_publisher = self.create_publisher(KeyValue, Topics.EXTENDED_SETTINGS_TOPIC, 10)
        self.settings_publisher   # prevent unused variable warning

        #Create timer for updates
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.update)


    def update(self):
        #Compare parameters with feedback from lolo and publish messages to lolo
        print("Checking for changed parameters")
        #Check all parameters and compare with lolos current status 
        for param in self.lolo_parameters:
            for key in dir(self.lolo_status_msg):
                #Find the variable the parameter corresponds to in the status message
                if(key == param.name):
                    #Get value from status message
                    status_value = getattr(self.lolo_status_msg, key)
                    if(status_value != param.value):
                        print("Found setting to change :" + str(key) + " currently=" + str(status_value) + " should be: " + str(param.value))
                        msg = KeyValue()
                        msg.key = str(param.name)
                        msg.value = str(param.value)
                        self.settings_publisher.publish(msg)
                        #print("\tMessage sent: " + str(msg))
                    else: 
                        print("Found setting with correct value :" + str(key) + " currently=" + str(status_value))

# The following is just to start the node
def main(args=None):
    rclpy.init(args=args)
    node = LoloParams()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    #rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()