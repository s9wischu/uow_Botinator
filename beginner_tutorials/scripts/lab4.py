#!/usr/bin/env python


from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, PoseWithCovarianceStamped
from std_msgs.msg import Header, ColorRGBA
import rospy
from geometry_msgs.msg import Twist
from math import *
from tf.transformations import euler_from_quaternion
import time

def init():
    global pub, count, moving_back, finished, current_time
    rospy.loginfo("started.")
    moving_back = False
    finished = False
    count = 0
    #current_time = time.time()
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, callback)
    rospy.spin()

current_time = 0

def callback(data):
    global count, init, current, returnangle, final, moving_back, finished, current_time
    #if current_time + 0.5 > time.time():
    #    return
    #current_time = time.time()
   
    
    #rospy.loginfo(str(current_time))

    rospy.loginfo("received a callback, count=" + str(count))
    rospy.loginfo("current angle = " + str(get_angle(data)))

    if finished:
        return

    count += 1
    current = data

    if count == 1:
        init = data
    elif count == 1000:
        final = data
        calc_return_angle()
        rotate_back()
    elif count > 1000:
        rotate_back()


def calc_return_angle():
    global init, current, returnangle, returndistance
    deltax = current.pose.pose.position.x - init.pose.pose.position.x
    deltay = current.pose.pose.position.y - init.pose.pose.position.y
    returnangle = atan2(-deltay, -deltax)
    returndistance = sqrt(deltax * deltax + deltay * deltay)

def get_angle(pose):
    orient = pose.pose.pose.orientation
    return euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])[2]

def rotate_back():    
    global pub, current, returnangle, moving_back
    msg = Twist()
    cur = get_angle(current)
    ret = returnangle
    error = abs(cur - ret)
    rospy.loginfo('cur ' + str(cur) + " ret " + str(ret) + " " + "Rotating. Angular error: " + str(error))
    
    if error > 0.1 and not moving_back:
        msg.angular.z = 1.0
        msg.angular.z = 1.0
        pub.publish(msg)
    else:
        moving_back = True
        move_back()
   
def move_back():
    global pub, current, returndistance, final
    deltax = final.pose.pose.position.x - current.pose.pose.position.x
    deltay = final.pose.pose.position.y - current.pose.pose.position.y
    dist_from_final = sqrt(deltax * deltax + deltay * deltay)
    msg = Twist()
    if dist_from_final < returndistance: 
        # reverse
        msg.linear.x = 0.1
    else:
        finished = True
        #msg.linear.x = 1 # was 0.25
    pub.publish(msg)

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
