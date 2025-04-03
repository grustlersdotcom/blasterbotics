import Jetson.GPIO as GPIO
import time
import socket
import threading

# Change as needed
MOVE_SPEED_CONSTANT = 100
FORWARD_BACKWARD_PIN = 15
LEFT_RIGHT_PIN = 18
ARM_RELAY_PIN_UP = 13 # Note we should change this to whatever it is later
ARM_RELAY_PIN_DOWN = 1 # NOTE replace this later
LIMIT_SWITCH_PIN = 15 # Note switch that here
WRIST_PIN = 1

# State Variables
current_position = None
target_position = None
arm_state = None

# Communications happen via sockets here
def pin_setup():
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(LEFT_RIGHT_PIN,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(FORWARD_BACKWARD_PIN,GPIO.OUT,initial=GPIO.LOW)
        fb_pwm = GPIO.PWM(FORWARD_BACKWARD_PIN,50)
        lr_pwm = GPIO.PWM(LEFT_RIGHT_PIN,50)
        GPIO.setup(ARM_RELAY_PIN_UP,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(LIMIT_SWITCH_PIN,GPIO.IN,initial=GPIO.LOW)
        GPIO.setup(ARM_RELAY_PIN_DOWN,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(WRIST_PIN,GPIO.OUT,initial=GPIO.LOW)
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

def limit_switch_callback():
    GPIO.output(ARM_RELAY_PIN_UP,GPIO.LOW)
    return

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
     GPIO.output(WRIST_PIN,GPIO.HIGH)
     return

# So we need to script out the triangle
# First we show the robot moving back and forward along the same line
# So we go backwards and forwards along some line
# Then we have the robot turn its wheels to some direction to get it to a new line
# Then we hard code the timer to make it so that at some predefined point so after a certiain amount of time we need to turn and then turn inverse to it
# Then we need to reorient itself aound a new line and then we need to lower the arm move forward some amount and then lift the arm.

def main():
    # First we setup all of the pins
    fb_pwm, lr_pwm = pin_setup()	
    GPIO.add_event_detect(LIMIT_SWITCH_PIN,GPIO.RISING,callback=limit_switch_callback,bouncetime=10,polltime=.1)
    print("Pins setup, limit switch callback registered")
    
    # Forward and backwards demo
    print("Demo of forward backwards motion")
    move_forward(fb_pwm=fb_pwm)
    time.sleep(0.5)
    stop_movement(fb_pwm=fb_pwm)
    move_backwards(fb_pwm=fb_pwm)
    time.sleep(0.5)
    stop_movement(fb_pwm=fb_pwm)

    # Arm up and down demo
    print("Demo of arm going up and down briefly")
    arm_up()
    time.sleep(.1)
    arm_down()
    time.sleep(.1)

    # Obstacle detection demo
    print("Demo of maneuvering around object & grabbing")
    # block of code to move the car along the new axis for the cone
    set_left_right_position(lr_pwm=lr_pwm,angle=70)
    move_forward(fb_pwm=fb_pwm)
    time.sleep(.1)
    set_left_right_position(lr_pwm=lr_pwm,angle=-70)
    time.sleep(.1)
    set_left_right_position(lr_pwm=lr_pwm,angle=0)
    # This is the block to maneuver around the cone
    time.sleep(.25)
    set_left_right_position(lr_pwm=lr_pwm,angle=50)
    time.sleep(.1)
    set_left_right_position(lr_pwm=lr_pwm,angle=-50)
    time.sleep(.1)
    stop_movement(fb_pwm=fb_pwm)
    arm_down()
    time.sleep(.1)
    move_forward(fb_pwm=fb_pwm)
    time.sleep(.1)
    stop_movement(fb_pwm=fb_pwm)
    arm_up()
    time.sleep(.1)
    turn_wrist()
    return

if __name__ == 'main':
	main()