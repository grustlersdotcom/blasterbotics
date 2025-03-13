import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import Jetson.GPIO as GPIO

# States:
# IDLE: Robot is doing nothing
# FORWARD: Robot is moving forward
# BACKWARD: Robot is Moving Backwards
# STOP: Robot is stopped

class MotorControl(Node):
    def pin_setup(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.LEFT_RIGHT_PIN,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.FORWARD_BACKWARD_PIN,GPIO.OUT,initial=GPIO.LOW)
        fb_pwm = GPIO.PWM(self.FORWARD_BACKWARD_PIN,50)
        lr_pwm = GPIO.PWM(self.LEFT_RIGHT_PIN,50)
        return fb_pwm,lr_pwm

    def move_forward(self):
        self.fb_pwm.ChangeDutyCycle(self.MOVE_SPEED_CONSTANT)
        return

    def stop_movement(self):
        self.fb_pwm.ChangeDutyCycle(0)
        return

    def set_lef_right_position(self):
        duty_cycle = angle / 180 * (10 + 2)
        self.lr_pwm.ChangeDutyCycle(duty_cycle)
        return

    #TODO Update this to matched published positionss
    def update_current_position(self,msg):
        self.current_position = msg
        return

    def state_machine_loop(self,msg):
        return

    def update_target_pos(self,msg):
        self.target_position = msg
        self.get_logger().info(f"Recieved target position: {msg}")
        return

    def __init__(self):
        super().__init__('motor_control')
        
        #Create needed publishers and subscribers
        self.gps_sub = self.create_subscription(
            String,
            'current_position',
            self.update_current_position,
            10
            )
        
        self.stop_sub = self.create_subscription(
            String,
            'stop_bot',
            self.stop_movement,
            10
            )

        self.target_pos_sub = self.create_subscription(
            String,
            'target_pos_sub',
            self.update_target_pos,
            10
            )

        # Setup state machine loop using a timer
        self.state_machine_time = self.create_timer(.1,self.state_machine_loop)

        #Setup needed pins
        self.fb_pwm, self.lr_pwm = self.pin_setup()
        
        # Initial state is to be idle, the pathfinding algorithm will update state
        self.state = "IDLE"

        # Set the needed Constants
        self.MOVE_SPEED_CONSTANT = 100
        self.FORWARD_BACKWARD_PIN = 15
        self.LEFT_RIGHT_PIN = 18

        # Store the current and target positions
        self.target_position = None
        self.current_position = None

        return
#TODO UPDATE MAIN
def main():
    print('Hi from bot.')


if __name__ == '__main__':
    main()
