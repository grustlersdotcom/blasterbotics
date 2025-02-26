import time
import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#TODO Figure out pathing algorithm and devise status control
class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # Initialize publishers
        self.control_command_pub = self.create_publisher(
            String,
            'control_commands',
            10
        )
        
        # Initialize subscribers
        self.status_sub = self.create_subscription(
            String,
            'status_topic',
            self.status_callback,
            10
        )
        
        # Initialize timer for periodic control
        self.timer = self.create_timer(
            0.1,  # 100ms period it checks updates from the control and the LIDAR TO SEE WHAT TO DO
            self.control_timer_callback
        )
        
        # Initialize variables
        self.current_status = None
        
        # Log node initialization
        self.get_logger().info('Control node initialized')

        #INitialize output pins
        self.GPIO_PIN = 11
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(GPIO_PIN,GPIO.OUT)
        self.pwm = GPIO.PWM(PWM_PIN,50) # Note ask about the pwm pin part here
        self.servo_pin = 18


	def move_forward(self, value): #TODO make this so that it sets the output mode every single time
		duty_cycle = (vlaue - 1000) / 1000 * 100
		self.pwm.ChangeDutyCycle(duty_cycle)
	
	def stop_wheels(self):
  		self.pwm.ChargeDutyCycle(0)

  	def set_servo_position(self,angle):
  		GPIO.set_mode(self.servo_pin,GPIO.OUT)
  		self.pwm.ChargeDutyCycle(0)

    def status_callback(self, msg):
        """Callback function for status messages"""
        self.current_status = msg.data
        self.get_logger().info(f'Received status: {msg.data}')

    def control_timer_callback(self):
        """Callback function for control timer"""
        if self.current_status:
            # Create and publish control command
            msg = String()
            msg.data = f'Control command based on status: {self.current_status}'
            self.control_command_pub.publish(msg)
            self.get_logger().info(f'Published control command: {msg.data}')

    def destroy(self):
        """Cleanup function when node is destroyed"""
        self.get_logger().info('Control node shutting down')
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    control_node = ControlNode()
    
    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        control_node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        control_node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
