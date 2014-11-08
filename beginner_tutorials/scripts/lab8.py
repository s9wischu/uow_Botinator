#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist

def callback(marker):
    global publisher
    print "Detected marker:", marker.id
    if marker.id == 5:
        if marker.pose.position.x < -0.05:
            twist = Twist()
            twist.angular.z = 0.75
            publisher.publish(twist)
        elif marker.pose.position.x > 0.05:
            twist = Twist()
            twist.angular.z = -0.75
            publisher.publish(twist)
    elif marker.id == 6:
        if marker.pose.position.x < 0:
            twist = Twist()
            twist.angular.z = -0.75
            publisher.publish(twist)
        elif marker.pose.position.x >= 0:
            twist = Twist()
            twist.angular.z = 0.75
            publisher.publish(twist)
    elif marker.id == 7:
        if marker.pose.position.x < -0.025:
            twist = Twist()
            twist.angular.z = 0.75
            publisher.publish(twist)
        elif marker.pose.position.x > 0.025:
            twist = Twist()
            twist.angular.z = -0.75
            publisher.publish(twist)
        elif marker.pose.position.z < 0.3:
            twist = Twist()
            twist.linear.x = -0.15
            publisher.publish(twist)
    elif marker.id == 8:
        if marker.pose.position.x < -0.025:
            twist = Twist()
            twist.angular.z = 0.75
            publisher.publish(twist)
        elif marker.pose.position.x > 0.025:
            twist = Twist()
            twist.angular.z = -0.75
            publisher.publish(twist)
        elif marker.pose.position.z < 1:
            twist = Twist()
            twist.linear.x = 0.15
            publisher.publish(twist)
    else:
        print "Warning: Unknown marker id!"


def main():
    global publisher
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('visualization_marker', Marker, callback)
    publisher= rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
