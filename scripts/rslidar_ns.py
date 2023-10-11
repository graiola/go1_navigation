#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

tf_prefix : str
pub : rospy.Publisher
sub : rospy.Subscriber

def callback(msg):
	new_msg = msg
	new_msg.header.frame_id = tf_prefix + "rslidar"
	pub.publish(new_msg)

if __name__ == '__main__':

	rospy.init_node("rslidar_ns")

	tf_prefix = rospy.get_param('~tf_prefix','')

	# Check and fix tf_prefix
	if tf_prefix == "/":
		tf_prefix = ""
	elif len(tf_prefix) != 0:
		tf_prefix = tf_prefix + "/"

	print('Launch rslidar_ns with tf_prefix="%s"' % (tf_prefix))
	
	if tf_prefix =='':
		print('tf_prefix is empty, no need to launch this script, exiting...')
		exit(0)
	
	pub = rospy.Publisher(tf_prefix+"rslidar_points", PointCloud2, queue_size=10)
	sub = rospy.Subscriber("rslidar_points", PointCloud2, callback)
	rospy.spin()
		