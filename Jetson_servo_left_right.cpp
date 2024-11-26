import jetson.gpio as GPIO

# Set GPIO mode
servo_pin = 18   # Attach the servo to pin 18.
gpio.set_mode(servo_pin,  gpio.OUT)

# Create PWM object with 50Hz frequency
pwm = gpio.PWM(servo_pin, 50)
pwm.start(0)

# Go to desired angle
set_servo_position(90)

def set_servo_position(angle):
  duty_cycle = angle / 180 * (10 + 2)
  pwm.ChangeDutyCycle(duty_cycle)

# Clean up GPIO
gpio.cleanup()

