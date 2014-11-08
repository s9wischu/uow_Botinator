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
	global pub
	global ignore
	global angular
	if ignore != 0: 
		ignore -= 1
		msg = Twist()
		msg.angular.z = angular
		pub.publish(msg)
		return

	data_out = pc2.read_points(data, field_names=None, skip_nans=True, uvs=[])

	# Number of buckets to use for y-coordinates 
	number_buckets = 1000
	# Threshold for points to be considered foreground
	factor = 0.85


	buckets = [[] for x in range(number_buckets)]
	
	for point in data_out:
		y = point[1]
		bucket = int((y + 1.0) / (2.0 / number_buckets))
		buckets[bucket].append(point)

	pointlist = []
	# Calculate the average for each bucket (if there are any points)
	for i in range(number_buckets):
		if(len(buckets[i]) == 0): continue
		
		# Calculate the average of all the z-coordinates in the bucket	
		average = 0.0
		for point in buckets[i]:
			average += point[2]
		average /= len(buckets[i])

		# Add the points with z-coordinates above the threshold (dependent
		# on average of z-coordinates)
		for point in buckets[i]:
			if point[2] < factor * average:
				pointlist.append(point)

		rospy.loginfo("average in bucket " + str(i) + ": " + str(average))

	
	# Iterate over all points in pointlist and select the point that is closest
        # to the camera (i.e., lowest z-coordinate). 
	minz = sys.maxint;
	closest_point = None	
	for point in pointlist:
		if point[2] < minz:
			minz = point[2]
			closest_point = point
	msg = Twist()
	if closest_point is None:
		rospy.loginfo("No point could be found."); 
		angular = 0
	else: 
		rospy.loginfo("Closest point found: (" + str(point[0]) + "," + str(point[1]) \
                              + ") with z-value " + str(point[3]))
		if point[0] < 0: 
			rospy.loginfo("it is on the left")
		else:
			rospy.loginfo("it is on the right")	
		
		if closest_point[0] < 0.5: 
			# Rotate to the right
			angular = +1
			ignore = 5
		elif closest_point[0] > 0.5: 
			# Rotate to the left
			angular = -1
			ignore = 5


        
if __name__ == '__main__':
	try:
		finder()
	except rospy.ROSInterruptException:
		pass

