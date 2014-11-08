#!/usr/bin/env python

from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
import rospy
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion



"""
Interface 1: 
"""

from Tkinter import *
import Tkinter
import Image, ImageTk

"""
class MyDialog:
    def __init__(self, parent):
        top = self.top = Toplevel(parent)
        Label(top, text="Value").pack()
        self.e = Entry(top)
        self.e.pack(padx=5)
        b = Button(top, text="OK", command=self.ok)
        b
"""
image_path = "/tmp/map2.pgm"

def point_callback(event):
    global dots, move, root
    
    # Transform pixel coordinates into map coordinates
    x = event.x / 20.0 - 1.0
    y = event.y / 20.0 - 12.2
    print("POINT: pixel (%d, %d), map coordinates: (%f, %f)" % (event.x, event.y, x, y))

    if move:
        _, xx, yy = closest_dot(x, y)
        publish(xx,yy)
    else:
        dots.append((False,x,y))
        draw_dots()
    root.destroy()

def region_callback(event):
    global dots, move, root
    
    # Transform pixel coordinates into map coordinates
    x = event.x / 20.0 - 1.0
    y = event.y / 20.0 - 12.2
    print("REGION: pixel (%d, %d), map coordinates: (%f, %f)" % (event.x, event.y, x, y))

    dots.append((True,x,y))
    draw_dots()
    root.destroy()

def draw_dots():
    global dots, tkimage, frame
    for dot in dots:
        r,g,b = (255,0,0) # red
        canvas = Canvas(frame)
        canvas.create_image(0, 0, tkimage)
        canvas.pack()

def closest_dot(x,y):
    global dots, root
    min_dist = float("Infinity")
    min_dot = None
    for dot in dots:
        dx = abs(x-dot[1])
        dy = abs(y-dot[2])
        d = dx*dx + dy*dy
        if d < min_dist:
            min_dist = d
            min_dot = dot
    return min_dot


def publish(x,y):
    global pub1,pub2,pub3
    print "Moving to", (x, y)

    # Publish move event
    twist = PoseStamped()
    twist.header.frame_id = 'map'
    twist.pose.position.x = x
    twist.pose.position.y = y
    twist.pose.orientation.z = 0.992267566594
    twist.pose.orientation.w = -0.124117187716

    pub1.publish(twist)
    pub2.publish(twist)
    pub3.publish(twist)
    
def display_map():
    global tkimage, root, frame

    im = Image.open(image_path)
    root = Tkinter.Tk()
    tkimage = ImageTk.PhotoImage(im)
    
    frame = Tkinter.Label(root, image=tkimage)

    frame.bind("<Button-1>", point_callback)
    frame.bind("<Button-3>", region_callback)
    frame.pack()

    draw_dots()

    root.mainloop()

if __name__ == "__main__":
    global pub1,pub2,pub3, dots, move
    dots = []

    rospy.init_node('listener_new_name_and_terminator', anonymous=True)

    pub1 = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    pub2 = rospy.Publisher('/move_base/goal', PoseStamped, queue_size=1)
    pub3 = rospy.Publisher('/move_base/current_goal', PoseStamped, queue_size=1)
    while True: 

        move_or_add = raw_input("MOVE (m) OR ADD REGIONS/POINTS (a): ")
        if move_or_add == "m":
            move = True
        elif move_or_add == "q":
            root.destroy()
            import sys
            sys.exit()
        else:
            move = False
       
        display_map()
