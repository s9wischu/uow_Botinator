#!/usr/bin/env python
# Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from random import randint

msg = Twist()
state = "MOVING"
counter = 0
RATE = 2 # hz MUST BE MULTIPLE OF 10
SPEED = 1.0

# states: HIT1, ROTATING, DEFAULT

def callback(data):
	global state
	if data.bumper == BumperEvent.PRESSED and state != "ROTATING":
		state = "HIT1"
		rospy.loginfo("CALLBACK state = %s" % state)
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        
if __name__ == '__main__':
	global state
	rospy.init_node('our_controller', anonymous=True)
	rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, callback)
    	pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

	r = rospy.Rate(RATE)
	while not rospy.is_shutdown():
		rospy.loginfo("state = %s" % state)
		if state == "HIT1":
			counter = 2
			state = "BACK"
			msg.linear.x = -SPEED
			msg.angular.z = 0.0
		
		if state == "BACK":
			counter -= 1
			if counter <= 0:
				# set a counter
				counter = randint(20,40)
				state = "ROTATING"
				msg.linear.x = 0.0
				msg.angular.z = 1.0

		if state == "ROTATING":
			counter -= 1
			if counter <= 0:
				state = "MOVING"

		if state == "MOVING":
			msg.linear.x = SPEED
			msg.angular.z = 0.0

#	rospy.loginfo("publishing in main(): msg = %s" % msg)
		pub.publish(msg)
		r.sleep()
