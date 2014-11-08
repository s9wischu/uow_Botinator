#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from random import randint
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import sys

def finder():
	global pub
	rospy.init_node('finder', anonymous=True)
	rospy.Subscriber("/camera/depth/points", PointCloud2, callback);
	pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
	rospy.spin()

ignore = 0
angular = 0

def callback(data):
	global ignore, angular 
	if ignore != 0:
		ignore -= 1;
		msg = Twist()
		msg.angular.z = angular
		pub.publish(msg);

	data_out = pc2.read_points(data, field_names=None, skip_nans=True, uvs=[])


	points = []
	for point in data_out:
		y = point[1]
		if 0.0 > y and y < 0.2:
			points.append(point)

	if len(points) == 0:
		rospy.loginfo("No points in strip.")
		angular = 0;
		return

	closest = None
	mindis = sys.maxint
	for point in points:
        	z = point[2]
		if mindis > z:
			closest = point
			mindis = z

	xpos = closest[0]
	if xpos < -0.5:
		rospy.loginfo("object is to the left")
		angular = 0.5
		ignore = 10
	elif xpos > 0.5:
		rospy.loginfo("object is to the right")
		angular = -.5
		ignore = 10
	else:
		rospy.loginfo("object is in the center")
		angular = 0


if __name__ == '__main__':
	try:
		finder()
	except rospy.ROSInterruptException:
		pass

