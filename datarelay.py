import rclpy
from rclpy.node import Node
from ghost_manager_interfaces.msg import Heartbeat
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
import threading
import time


class HeartbeatListener(Node):
    def __init__(self):
        super().__init__('heartbeat_listener')

        # Message data holders
        self.str_heartbeat_estop = 0
        self.str_heartbeat_ctrlMode = 0
        self.str_stm32 = ""
        self.flt_linearX = 0.0
        self.int_counter = 0;

        # Create publisher
        self.publisher_String = self.create_publisher(String, '/vision60_toSTM32', 10)
        self.publisher_ForcedForward = self.create_publisher(String, '/stm32_input', 10)

        # Subscriptions
        self.create_subscription(Heartbeat, '/state/heartbeat', self.heartbeat_listener, 10)
        self.create_subscription(String, '/stm32_input', self.stm32_listener, 10)
        self.create_subscription(TwistStamped, '/mcu/state/vel', self.velocityFeedBack_listener, 10)

        # Start validation thread
        self.validation_thread = threading.Thread(target=self.validation_loop)
        self.validation_thread.daemon = True  # Auto-stop with main thread
        self.validation_thread.start()

    def heartbeat_listener(self, msg):
        self.str_heartbeat_estop = msg.estop
        self.str_heartbeat_ctrlMode = msg.control_mode

    def stm32_listener(self, msg):
        self.str_stm32 = msg.data

    def velocityFeedBack_listener(self, msg):
        self.flt_linearX = msg.twist.linear.x

    def publish_toSTM32(self, input_data):
        msg = String()
        msg.data = input_data
        self.publisher_String.publish(msg)

    def publish_ForcedForward(self, input_data):
        msg = String()
        msg.data = input_data
        self.publisher_ForcedForward.publish(msg)

    def validation_loop(self):
        while rclpy.ok():
            # Check conditions
            if self.str_heartbeat_estop == 1 or self.str_heartbeat_ctrlMode == 180:
                self.get_logger().info("Publishing estop to /vision60_toSTM32")
                self.publish_toSTM32("estop")
            else:
                if self.str_stm32 == "forward" and self.flt_linearX < 1.00:
                    self.get_logger().info("Condition met: test input + velocity < 1.0")
                    self.int_counter = self.int_counter + 1
                    
                    if self.int_counter == 5:
                    	Obstacle = True
                    	self.publish_toSTM32("stop")
                    	print("started obstacle avoidance loop")
                    	
                    	while Obstacle == True:
                    		self.publish_ForcedForward("forward")
                    		print("publishing forward")
                    		if self.flt_linearX >= 1:
                    			Obstacle = False
                    			self.int_counter = 0
                    			self.publish_toSTM32("continue")
                    			print("continue!")
                    		else:
                    			self.publish_toSTM32("stop")
                    			print("obstacle still there")
                    
                else:
                	self.int_counter = 0
                	
                	print("Didnt hit 5 so resetting counter to 0 xD")

            time.sleep(0.1)  # Check every 100ms

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

