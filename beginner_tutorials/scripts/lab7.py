#!/usr/bin/env python

import roslib; roslib.load_manifest("interactive_markers")
import rospy
import copy

from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

from random import random
from math import sin

server = None
marker_pos = 0
menu_handler = MenuHandler()

# TODO if we click on a point/region marker the robot should go to it
def processFeedback( feedback ):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo( s + ": button click" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo( s + ": pose changed")
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo( s + ": mouse down" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo( s + ": mouse up" + mp + "." )
    server.applyChanges()

def alignMarker( feedback ):
    pose = feedback.pose

    pose.position.x = round(pose.position.x-0.5)+0.5
    pose.position.y = round(pose.position.y-0.5)+0.5

    rospy.loginfo( feedback.marker_name + ": aligning position = " + str(feedback.pose.position.x) + "," + str(feedback.pose.position.y) + "," + str(feedback.pose.position.z) + " to " +
                                                                     str(pose.position.x) + "," + str(pose.position.y) + "," + str(pose.position.z) )

    server.setPose( feedback.marker_name, pose )
    server.applyChanges()

def rand( min_, max_ ):
    return min_ + random()*(max_-min_)

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def saveMarker( int_marker ):
  server.insert(int_marker, processFeedback)


#####################################################################
# Marker Creation

def make_marker(typ,x,y,t,name):
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/map"
    int_marker.pose.position.x = x
    int_marker.pose.position.y = y
    # TODO: theta int_marker.orientation
    marker_pos += 1
    int_marker.scale = 1

    int_marker.name = name + ", type = " + typ
    int_marker.description = int_marker.name

    # insert a box
    makeBoxControl(int_marker)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)

if __name__=="__main__":
    rospy.init_node("basic_controls")
    
    server = InteractiveMarkerServer("basic_controls")

    menu_handler.insert( "First Entry", callback=processFeedback )
    menu_handler.insert( "Second Entry", callback=processFeedback )
    sub_menu_handle = menu_handler.insert( "Submenu" )
    menu_handler.insert( "First Entry", parent=sub_menu_handle, callback=processFeedback )
    menu_handler.insert( "Second Entry", parent=sub_menu_handle, callback=processFeedback )

    path = "/tmp/E003"
    with open(path, "r") as f:
        for line in f.readlines():
            parts = line.split(' ', 4)
            typ = parts[0]
            pixel_x = int(parts[1])
            x = pixel_x/20.0 - 1.0
            pixel_y = int(parts[2])
            y = pixel_y/20.0 - 12.2
            t = float(parts[3])
            n = parts[4]
            print typ, x, y, t, n
            make_marker(typ,x,y,t,n)

    server.applyChanges()

    rospy.spin()


