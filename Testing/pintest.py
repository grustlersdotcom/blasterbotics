import Jetson.GPIO as GPIO
import time

FORWARD_BACKWARD_PIN = 15
LEFT_RIGHT_PIN = 18

MOVE_SPEED_CONSTANT = 100

def setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(LEFT_RIGHT_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(FORWARD_BACKWARD_PIN, GPIO.OUT, initial=GPIO.LOW)
    fb_pwm = GPIO.PWM(FORWARD_BACKWARD_PIN, 50)
    lr_pwm = GPIO.PWM(LEFT_RIGHT_PIN, 50)
    fb_pwm.start(0)
    lr_pwm.start(0)
    return fb_pwm, lr_pwm


def stop_movement(fb_pwm):
    fb_pwm.ChangeDutyCycle(0)
    time.sleep(0.1)
    return


def move_forward(fb_pwm):
    fb_pwm.ChangeDutyCycle(MOVE_SPEED_CONSTANT)
    return


def move_backward(fb_pwm):
    fb_pwm.ChangeDutyCycle(-1 * MOVE_SPEED_CONSTANT)
    return


# Note this function assumes that right is positive and left is negative
def set_left_right_position(angle, lr_pwm):
    duty_cycle = angle / 180 * (10 + 2)
    lr_pwm.ChangeDutyCycle(duty_cycle)
    return


fb_pwm, lr_pwm = setup()

# Perform movements
move_forward(fb_pwm)
set_left_right_position(78, lr_pwm)

# Stop movements before cleanup
stop_movement(fb_pwm)

# Properly stopping the PWM before cleanup
fb_pwm.stop()
lr_pwm.stop()

# Cleanup GPIO properly after stopping PWM
#GPIO.cleanup()
