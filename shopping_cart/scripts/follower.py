#!/usr/bin/env python

import rospy

from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from kobuki_msgs.msg import BumperEvent
import threading

class Struc:
    publisher = None
    THRESHOLD = 0.80
    MAX_DISTANCE = 0.75
    SPEED_X = 0.2
    SPEED_Z = 0.75
    MIN_DISTANCE = 0.25
    currentPose = None
    currentAngle = 0.0
    legSubscriber = None
    odomSubscriber = None
    bumperSubscriber = None
    lostPublisher = None
    bumperPublisher = None
    currentLinSpeed = 0.0
    currentAngSpeed = 0.0
    deltaLinSpeed = 0.0
    
    running = False
    lastPerson = None
    lastMeasurement = 0

    measurementsSinceLegLoss = 0
    
def onLegMeasurement(data):
    from math import atan2, sqrt, pi
 
    if len(data.people) == 0:
        return
        
    if Struc.currentPose == None: 
        return
  

    if Struc.lastPerson is None:
        closest = 10000000
        for d in data.people:
            dx = d.pos.x - Struc.currentPose.x  
            dy = d.pos.y - Struc.currentPose.y
            distance = dx * dx + dy * dy
            
            if d.reliability > Struc.THRESHOLD and distance < closest:
                closest = distance
                Struc.lastPerson = d
                print "Following new legs", d.object_id

    person = None

    if Struc.lastPerson is not None:
        for d in data.people:
            if d.object_id == Struc.lastPerson.object_id:
                person = d
                break
    # Find closest legs that were detected wtih high reliability
   
            
    if person is None and Struc.lastPerson is not None:
        closest = 10000000
        closestPerson = None
        for d in data.people:
            dx = d.pos.x - Struc.lastPerson.pos.x  
            dy = d.pos.y - Struc.lastPerson.pos.y
            distance = dx * dx + dy * dy

            if distance < closest:
                closest = distance
                closestPerson = d
                
        if closest < Struc.MIN_DISTANCE and closestPerson is not None:
            person = closestPerson
            Struc.lastPerson = person
            print "\t\t\t\t\tNew id:", person.object_id
            
    if person is None:
        Struc.measurementsSinceLegLoss += 1
        if Struc.measurementsSinceLegLoss <= 2:
            print "IGNORED"
            return
        
        print "discarded"
        Struc.lostPublisher.publish(True)
        stopFollow()
        return

    Struc.measurementsSinceLegLoss = 0

    print "following", person.object_id
        
    print "Reliability",    person.reliability     
            
    Struc.lastMeasurement = 0
       
    dx = person.pos.x - Struc.currentPose.x  
    dy = person.pos.y - Struc.currentPose.y
    distance = sqrt(dx * dx + dy * dy)
 
    personangle = atan2(dy, dx)
    angle = personangle - Struc.currentAngle
    
    print "distance:", distance, "angle:", angle
    
    if angle > 0.15:
        print "left"
        Struc.currentAngSpeed = +Struc.SPEED_Z
    elif angle < -0.15:
        print "right"
        Struc.currentAngSpeed = -Struc.SPEED_Z
    else:
        print "no angular"
        Struc.currentAngSpeed = 0
        
           
    if distance > Struc.MAX_DISTANCE:
        print "forward"
        msg = Twist()
        Struc.deltaLinSpeed = Struc.SPEED_X
    else:
        Struc.deltaLinSpeed = -Struc.SPEED_X
        print "no linear"
    
    print "current speed", Struc.currentAngSpeed, Struc.currentLinSpeed
    
    
def onBumperEvent(data):
    Struc.bumperPublisher.publish(True)
    stopFollow()

def onOdomMeasurement(data):
    from tf.transformations import euler_from_quaternion
    Struc.currentPose = data.pose.pose.position
    orient = data.pose.pose.orientation
    Struc.currentAngle = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])[2]
    
def onFollowModeTrigger(mode):
    if mode.data:
        startFollow()
    else:
        stopFollow()
    
def startFollow():
    print "\t\t\tstart leg following"
    Struc.legSubscriber = rospy.Subscriber("/leg_tracker_measurements", PositionMeasurementArray, onLegMeasurement)
    Struc.odomSubscriber = rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, onOdomMeasurement)
    Struc.publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
    Struc.bumperSubscriber = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, onBumperEvent)
    Struc.running = True
    Struc.currentPerson = None
    threading.Timer(0.05, sendMovementCommand).start()
    Struc.initialPosition = Struc.currentPose
    #Struc.lostPublisher.publish(False)   
 
def stopFollow():
    print "\t\t\tstop leg following"
    Struc.lastPerson = None
    Struc.running = False
    Struc.currentLinSpeed = 0.0
    Struc.currentAngSpeed = 0.0
    Struc.deltaLinSpeed = 0.0

    Struc.legSubscriber.unregister() 
    Struc.odomSubscriber.unregister()
    Struc.bumperSubscriber.unregister()    
    print "\n\n\n\t\t\t\tLost Triggered\n\n\n"
    
 
def sendMovementCommand():
    if not Struc.running:
        return
    threading.Timer(0.001, sendMovementCommand).start()
    
    # Stop if there hasn't been a measurement for a long time
    if Struc.lastMeasurement > 500:
        Struc.currentAngSpeed = 0
        Struc.deltaLinSpeed = -Struc.SPEED_X
        return
    
    Struc.currentLinSpeed += Struc.deltaLinSpeed / 500.0
    
    Struc.deltalLinSpeed = 0.0
    
    if Struc.currentLinSpeed > Struc.SPEED_X:
        Struc.currentLinSpeed = Struc.SPEED_X
    if Struc.currentLinSpeed < 0:
        Struc.currentLinSpeed = 0
    
    msg = Twist()
    msg.angular.z = Struc.currentAngSpeed
    msg.linear.x = Struc.currentLinSpeed
    Struc.publisher.publish(msg)
    
    Struc.lastMeasurement += 1


if __name__ == "__main__":
    rospy.init_node('follower', anonymous=True)
    rospy.Subscriber("/ours/followModeTrigger", Bool, onFollowModeTrigger)
    Struc.lostPublisher = rospy.Publisher('/ours/lostModeTrigger', Bool)
    Struc.bumperPublisher = rospy.Publisher('/ours/bumperModeTrigger', Bool)
    rospy.spin()
    
