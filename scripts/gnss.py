#!/usr/bin/env python

import serial
import time

import rospy
from sensor_msgs.msg import NavSatFix

import pdb

# Configure the serial port ex: '/dev/ttyUSB0'
ser = serial.Serial('/dev/ttyUSB2', baudrate=115200, timeout=1)


def send_command(command):
	ser.write(command.encode() + b'\r\n')
	time.sleep(1)
	return ser.readlines()


if __name__ == '__main__':

	rospy.init_node("gps_fix_pub")

	tf_prefix = rospy.get_param('~tf_prefix','')

	# Check and fix tf_prefix
	if tf_prefix == "/":
		tf_prefix = ""
	elif len(tf_prefix) != 0:
		tf_prefix = tf_prefix + "/"

	print('Launch gps_fix_pub with tf_prefix="%s"' % (tf_prefix))
	
	gps_pub = rospy.Publisher(tf_prefix+"gps/fix", NavSatFix, queue_size=10)

	send_command('AT')

	# Activate GPS module
	send_command('AT+QGPS=1')

	gps_msg = NavSatFix()
	gps_msg.header.frame_id = tf_prefix+"trunk"

	while not rospy.is_shutdown():
		# Get GPS coordinates
		response = send_command('AT+QGPSLOC=1')
		# pdb.set_trace()
		latitude = ""
		longitude = ""
		altitude = ""

		# Read the answer and search for GPS coordinates
		for line in response:
			line = line.decode().strip()
			if line.startswith('+QGPSLOC'):
				parts = line.split(',')
				if len(parts) >= 6:
					latitude = parts[1]
					longitude = parts[3]
					altitude = parts[5]
					print('Latitude: %s, Longitude: %s, Altitude: %s' % (latitude, longitude, altitude))

		if latitude != "":
			gps_msg.latitude = float(latitude) / 100
			gps_msg.longitude = float(longitude) / 100
			gps_msg.altitude = float(altitude)
			gps_msg.header.stamp = rospy.Time.now()

			gps_pub.publish(gps_msg)
	# Close the serial port
	ser.close()
