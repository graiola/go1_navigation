#!/usr/bin/env python

"""
read GPS raw from PX4
"""

from __future__ import print_function
from pymavlink import mavutil
import rospy
from sensor_msgs.msg import NavSatFix

"""
Mavlink messages in https://mavlink.io/en/messages/common.html

The GPS_RAW_INT msg has the following attributes:
time_usec : 3396340374, 
fix_type : GPS fix type.
lat : Latitude (WGS84, EGM96 ellipsoid) # degE7
lon : Longitude (WGS84, EGM96 ellipsoid) # degE7
alt : Altitude (MSL). Positive for up. # mm 
eph : GPS HDOP vertical dilution of position (unitless * 100)
epv : GPS VDOP vertical dilution of position (unitless * 100)
vel : GPS ground speed. # cm/s
cog : Course over ground #cdeg
satellites_visible : Number of satellites visible. If unknown, set to UINT8_MAX
alt_ellipsoid : # mm
h_acc : Position uncertainty # mm
v_acc : Altitude uncertainty # mm
vel_acc : Speed uncertainty # mm
hdg_acc : Heading / track uncertainty # degE5
yaw : Yaw in earth frame from north # cdeg
"""

# Check https://docs.px4.io/main/en/companion_computer/pixhawk_companion.html
device = "/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00"
baudrate = 57600

if __name__ == '__main__':

    rospy.init_node("mavgps_pub")

    tf_prefix = rospy.get_param('~tf_prefix', '')

    # Check and fix tf_prefix
    if tf_prefix == "/":
        tf_prefix = ""
    elif len(tf_prefix) != 0:
        tf_prefix = tf_prefix + "/"

    print('Launch mavgps_pub with tf_prefix="%s"' % (tf_prefix))

    gps_pub = rospy.Publisher(tf_prefix + "gps/fix", NavSatFix, queue_size=10)

    mlog = mavutil.mavlink_connection(device, autoreconnect=True, baud=baudrate)

    gps_msg = NavSatFix()
    gps_msg.header.frame_id = tf_prefix + "trunk"

    while not rospy.is_shutdown():
    
        msg = mlog.recv_match(type="GPS_RAW_INT", blocking=False)

        if msg is not None:
            print('Latitude: %s, Longitude: %s, Altitude: %s' % (msg.lat, msg.lon, msg.alt))
            gps_msg.latitude = float(msg.lat) / 10000000
            gps_msg.longitude = float(msg.lon) / 10000000
            gps_msg.altitude = float(msg.alt) / 1000
            gps_msg.header.stamp = rospy.Time.now()
            gps_pub.publish(gps_msg)
