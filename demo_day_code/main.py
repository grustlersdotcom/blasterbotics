import Jetson.GPIO as GPIO
import time
import socket
import threading
from ublox_gps import UbloxGps
import serial

# Change as needed
MOVE_SPEED_CONSTANT = 100
FORWARD_BACKWARD_PIN = 15
LEFT_RIGHT_PIN = 18

# Communication Flags
stop_flag = False

# State Variables
current_position = None
target_position = None

# Communications happen via sockets here
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

#TODO have this run in a thread
def listen_for_messages():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 8080))  # Listening on port 8080
    server_socket.listen(1)
    global stop_flag

    print("Python application waiting for messages...")

    while True:
        client_socket, _ = server_socket.accept()
        message = client_socket.recv(1024).decode('utf-8')
        if message:
            print("Received message:", message)
            # Change flag or perform action based on the message here
            if message == "Obstacle Detected":
                print("Flag set in Python application")
                stop_flag = True
        client_socket.close()

# define a function that reads in coordinates from GPS
#TODO make this run in its own thread
def poll_gps():
	port = serial.Serial('dev/serial10',baudrate=38400,timeout=1)
	gps = UbloxGps()
	global current_position

	while(True):
		try:
			coords = gps.geo_coords()
			current_position = (coords.lon,coords.lat)
		except (ValueError,IOError) as err:
			print(err)
		finally:
			port.close()
	return

def main():
	fb_pwm, lr_pwm = pin_setup()
	

	while !stop_flag:
		# Find a way to 
		pass
	return

if __name__ == 'main':
	main()