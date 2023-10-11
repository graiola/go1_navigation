#!/usr/bin/env python3

"""
read GPS raw from PX4
"""
from __future__ import print_function
from pymavlink import mavutil

device = "/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00"
baudrate = 57600

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

mlog = mavutil.mavlink_connection(device, autoreconnect=True, baud=baudrate)

while True:
    
    m = mlog.recv_match(type="GPS_RAW_INT", blocking=True)

    if m is not None:
        print(m)
        print("\n")
        print(m.lat)
        print("\n")
