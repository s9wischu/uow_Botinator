#!/usr/bin/env python

## read feducials and creat depending on the feducial number Scan_Items
## which means we have an item interaction or Scan_User which means we have
## an user interaction

import rospy
from beginner_tutorials.msg import Scan_Item
from beginner_tutorials.msg import Scan_User
from ar_track_alvar.msg import AlvarMarkers
from visualization_msgs.msg import Marker

pubItem = 0
pubUser = 0
# fiducial 1 to itemStart are for customer interaction
# fiducial itemStart to (inf) are for item interaction
itemStart = 8

def callback(scan):
    global pubItem, pubUser
    
    if scan.id < itemStart: # publish user scan
        item = Scan_Item()
        item.item = scan.id
        pubItem.publish(item)
        #rospy.loginfo("item")
    else: # publish item scan
        user = Scan_User()
        user = scan.id
        pubUser.publish(user)
        #rospy.loginfo("user")


def scanner():
    global pubItem, pubUser
    pubItem = rospy.Publisher('scan_item', Scan_Item, queue_size=10)
    pubUser = rospy.Publisher('scan_user', Scan_User, queue_size=10)
    
    rospy.init_node('scanner', anonymous=True)
    rospy.Subscriber('visualization_marker', Marker, callback)
    
    rospy.spin()
   

        
if __name__ == '__main__':
    try:
        scanner()
    except rospy.ROSInterruptException: pass
