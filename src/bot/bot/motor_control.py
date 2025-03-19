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

     def state_machine_loop(self):
        # First check if the robot is instructed to stop
        if self.state == "STOP":
            self.stop_movement()  # Ensure that we stop if the state is STOP

        #TODO add tolerances here, add check for turnarounds
        elif self.target_position is not None and self.current_position is not None:
            if self.current_position < self.target_position:
                self.state = "FORWARD"
            elif self.current_position > self.target_position:
                self.state = "BACKWARD"
            else:
                self.state = "STOP"  # If at target, stop the robot

        # Based on the state, perform the corresponding action
        if self.state == "IDLE":
            self.stop_movement()
        elif self.state == "FORWARD":
            self.move_forward()
        elif self.state == "BACKWARD":
            self.move_backward()
        elif self.state == "STOP":
            self.stop_movement()
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

        # Setup state machine loop using a timer
        self.state_machine_time = self.create_timer(.1,self.state_machine_loop)
        return

    def cleanup(self):
        #Note cleanup is not necessary here due to the stopping of the pwm individually
        self.fb_pwm.stop()
        self.lr_pwm.stop()
        return

def main():
    rclpy.init()

    motor_control = MotorControl()

    try:
        rclpy.spin(motor_control)
    except KeyboardInterrupt:
        pass
    finally:
        motor_control.cleanup()
        rclpy.shutdown()




if __name__ == '__main__':
    main()
