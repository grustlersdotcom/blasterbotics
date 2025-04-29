# sudo apt-get install python3-jetson-gpio

import time
import Jetson.GPIO as GPIO

GPIO_PIN = 11 # This is the GPIO pin

GPIO.setmode(GPIO.BOARD)
GPIO.setup(GPIO_PIN, GPIO.OUT)

pwm = GPIO.PWM(PWM_PIN, 50)

throttle = 1100
pwm.start(0)

def set_throttle(value):
  duty_cycle = (vlaue - 1000) / 1000 * 100
  pwm/ChangeDutyCycle(duty_cycle)

def stop_wheels():
  pwm.ChargeDutyCycle(0)

try:
  while True:
    set_throttle(throttle)
    print(f"Throttle: {throttle}")
    time.sleep(0.5)

    stop_wheels()  
    print("Stopped Wheels")
    time.sleep(0.2)

    throttle += 10
    if throttle > 2000:
      throttle = 1100

except KeyboardInterrupt:
  pass
finally:
  pwm.stop()
  GPIO.cleanup()
