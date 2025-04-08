import Jetson.GPIO as GPIO
import time


#TODO ADD WRIST CALLBACKS

# Change as needed
MOVE_SPEED_CONSTANT = 100
FORWARD_BACKWARD_PIN = 15
LEFT_RIGHT_PIN = 18
ARM_RELAY_PIN_UP = 13 # Note we should change this to whatever it is later
ARM_RELAY_PIN_DOWN = 1 # NOTE replace this later
LIMIT_SWITCH_PIN_UP = 15 # Note switch that here
LIMIT_SWITCH_PIN_DOWN = 1
WRIST_PIN_CW = 1
WRIST_PIN_CCW = 1

# State Variables
current_position = None
target_position = None
arm_state = None


# TODO finish the pin setups here
# Communications happen via sockets here
def pin_setup():
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(LEFT_RIGHT_PIN,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(FORWARD_BACKWARD_PIN,GPIO.OUT,initial=GPIO.LOW)
        fb_pwm = GPIO.PWM(FORWARD_BACKWARD_PIN,50)
        lr_pwm = GPIO.PWM(LEFT_RIGHT_PIN,50)
        GPIO.setup(ARM_RELAY_PIN_UP,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(LIMIT_SWITCH_PIN_UP,GPIO.IN,initial=GPIO.LOW)
        GPIO.setup(ARM_RELAY_PIN_DOWN,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(WRIST_PIN_CW,GPIO.OUT,initial=GPIO.LOW)
        return fb_pwm,lr_pwm

def move_forward(fb_pwm):
    fb_pwm.ChangeDutyCycle(MOVE_SPEED_CONSTANT)
    return

def stop_movement(fb_pwm):
    fb_pwm.ChangeDutyCycle(0)
    return

def move_backwards(fb_pwm):
    fb_pwm.ChangeDutyCycle(-1 * MOVE_SPEED_CONSTANT)
    return

def set_left_right_position(lr_pwm,angle):
    duty_cycle = angle / 180 * (10 + 2)
    lr_pwm.ChangeDutyCycle(duty_cycle)
    return

def limit_switch_up_callback():
    GPIO.output(ARM_RELAY_PIN_UP,GPIO.LOW)
    return

def limit_switch_down_callback():
    GPIO.output(ARM_RELAY_PIN_DOWN,GPIO.LOW)

def arm_up():
    arm_state = "UP"
    GPIO.output(ARM_RELAY_PIN_UP,GPIO.HIGH)
    return

def arm_down():
    arm_state = "DOWM"
    GPIO.output(ARM_RELAY_PIN_DOWN,GPIO.HIGH)
    return

def stop_arm():
    if arm_state == "UP":
        GPIO.output(ARM_RELAY_PIN_UP,GPIO.LOW)
    elif arm_state == "DOWN":
        GPIO.output(ARM_RELAY_PIN_DOWN,GPIO.LOW)
    return

def turn_wrist():
     GPIO.output(WRIST_PIN_CW,GPIO.HIGH)
     return

# So we need to script out the triangle
# First we show the robot moving back and forward along the same line
# So we go backwards and forwards along some line
# Then we have the robot turn its wheels to some direction to get it to a new line
# Then we hard code the timer to make it so that at some predefined point so after a certiain amount of time we need to turn and then turn inverse to it
# Then we need to reorient itself aound a new line and then we need to lower the arm move forward some amount and then lift the arm.

INITIAL_LINE_TEST_CONST = True
SCOOP_TEST_CONST = False
TURN_TEST_CONST = False
SWERVE_TEST_CONST = False
DUMP_TEST_CONST = False

def main():
    # First we setup all of the pins
    fb_pwm, lr_pwm = pin_setup()	
    GPIO.add_event_detect(LIMIT_SWITCH_PIN_UP,GPIO.RISING,callback=limit_switch_up_callback,bouncetime=10,polltime=.1)
    #TODO ADD LIMITSWITCH DOWNN EVENT DETECT
    print("Pins setup, limit switch callback registered")
    
    # Move forwards on the initial line
    if INITIAL_LINE_TEST_CONST:
        move_forward(fb_pwm=fb_pwm)
        time.sleep(1)
        stop_movement(fb_pwm=fb_pwm)
    # Perform the scoop
    if SCOOP_TEST_CONST:
        arm_down()
        move_forward(fb_pwm=fb_pwm)
        time.sleep(.01)
        stop_movement(fb_pwm=fb_pwm)
        arm_up()
    # Now we need to perform the turn
    if TURN_TEST_CONST:
        set_left_right_position(lr_pwm=lr_pwm,angle=70)
        move_forward(fb_pwm=fb_pwm)
        time.sleep(3)
    # We should be on the new axis now
    if SWERVE_TEST_CONST:
        set_left_right_position(lr_pwm=lr_pwm,angle=0)
        time.sleep(1) # Sleep to before we need to swerve
        set_left_right_position(lr_pwm=lr_pwm,angle=50)
        time.sleep(.1)
        set_left_right_position(lr_pwm=lr_pwm,angle=-50)
        time.sleep(.1)
        set_left_right_position(lr_pwm=lr_pwm,angle=0)
    # Obstacle should be passed so now we can just go to the dump site
    if DUMP_TEST_CONST:
        time.sleep(1)
        stop_movement(fb_pwm=fb_pwm)
        # Turn the wrist
        turn_wrist()
        time.sleep(.1)
    return

if __name__ == 'main':
	main()