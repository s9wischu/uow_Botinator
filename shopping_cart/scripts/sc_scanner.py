#!/usr/bin/env python

## read fiducials and create Scan_Items (depending on the fiducial number)
## which means we have an item interaction or Scan_User which means we have
## an user interaction

import rospy
from std_msgs.msg import String
from ar_track_alvar.msg import AlvarMarkers
from visualization_msgs.msg import Marker


import server
import structures

pubItem = 0
pubUser = 0
# fiducial 1 to itemStart are for customer interaction
# fiducial itemStart to (inf) are for item interaction
itemStart = 8

def callback(scan):
    global lastFiducialTime
    from time import time
    
    TIME_BETWEEN_SCANS = 2.5
    
    # Make sure that fiducials are not registered too often
    if time() - lastFiducialTime > TIME_BETWEEN_SCANS:
        server.ServerData.controller.onFiducial(scan.id)
        lastFiducialTime = time()
   

def scanner():
    global lastFiducialTime
    
    lastFiducialTime = 0
    
    rospy.init_node('scanner', anonymous=True)
    rospy.Subscriber('visualization_marker', Marker, callback)
    
    rospy.spin()
   

        
if __name__ == '__main__':
    try:
        scanner()
    except rospy.ROSInterruptException: pass
