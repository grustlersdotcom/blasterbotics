import Jetson.GPIO as GPIO
import time

MOVE_SPEED_CONSTANT = 100
FORWARD_BACKWARD_PIN = 15
LEFT_RIGHT_PIN = 18

stop_flag = False

def pin_setup():
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(LEFT_RIGHT_PIN,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(FORWARD_BACKWARD_PIN,GPIO.OUT,initial=GPIO.LOW)
        fb_pwm = GPIO.PWM(FORWARD_BACKWARD_PIN,50)
        lr_pwm = GPIO.PWM(LEFT_RIGHT_PIN,50)
        return fb_pwm,lr_pwm

def move_forward(fb_pwm):
    fb_pwm.ChangeDutyCycle(MOVE_SPEED_CONSTANT)
    return

def stop_movement(fb_pwm):
    fb_pwm.ChangeDutyCycle(0)
    return

def set_left_right_position(lr_pwm):
    duty_cycle = angle / 180 * (10 + 2)
    lr_pwm.ChangeDutyCycle(duty_cycle)
    return

def main():
	fb_pwm, lr_pwm = pin_setup()
	while !stop_flag
	return

if __name__ == 'main':
	main()