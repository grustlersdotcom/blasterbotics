import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from ublox_gps import UbloxGps

def gps_poller(Node):
	def __init__(self):
		#Node update this when GPS is connected
		self.port = serial.Serial('dev/serial10',baudrate=38400,timeout=1)
		self.gps = UbloxGps(port)

		self.current_loc_pub = self.create_publisher(
			String,
			'current_position'
			)
		# Now we create a timer to poll the location
		self.poll_time = self.create_timer(.1,self.grab_location)
		return
	
	def grab_location(self):
		coords = self.gps.geo_coords()
		msg = String()
		msg.data = f'{coords.lon},{coords.lat}'
		self.current_loc_pub.publish(msg)
		return

def main():
	return

if __name__ == 'main':
	main()