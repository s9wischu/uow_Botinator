#!/usr/bin/env python

from Tkinter import *
from ttk import Style
from PIL import Image, ImageTk
import rospy
from geometry_msgs.msg import *

class DialogSave:
    def __init__(self, parent, text):
        self.top = Toplevel(parent)
        Label(self.top, text=text).pack()
        Label(self.top, text="Name of file:").pack()
        self.entry = Entry(self.top)
        self.entry.pack(padx=5)
        b = Button(self.top, text="OK", command=self.ok)
        b.pack(pady=5)
 
    def ok(self):
        self.filename = self.entry.get()
        self.top.destroy()
 
class DialogPose:
    def __init__(self, parent, text):
        self.top = Toplevel(parent)
        Label(self.top, text=text).pack()
        Label(self.top, text="Name:").pack()
        self.entry = Entry(self.top)
        self.entry.pack(padx=5)
        Label(self.top, text="Theta:").pack()
        self.entryTheta = Entry(self.top)
        self.entryTheta.pack(padx=5)
        b = Button(self.top, text="OK", command=self.ok)
        b.pack(pady=5)
 
    def ok(self):
        self.theta = float(self.entryTheta.get())
        self.text = self.entry.get()
        self.top.destroy()
 
class DialogRegion:
    def __init__(self, parent, text):
        self.top = Toplevel(parent)
        Label(self.top, text=text).pack()
        Label(self.top, text="Name:").pack()
        self.entry = Entry(self.top)
        self.entry.pack(padx=5)
        Label(self.top, text="Radius:").pack()
        self.entryNum = Entry(self.top)
        self.entryNum.pack(padx=5)
        b = Button(self.top, text="OK", command=self.ok)
        b.pack(pady=5)
 
    def ok(self):
        self.text = self.entry.get()
        self.radius = float(self.entryNum.get())
        self.top.destroy()
 
class MainFrame(Frame):
    def __init__(self, parent, communicator):
        Frame.__init__(self, parent)
        self.parent = parent
        self.selected = None
        self.state = "NONE"
        self.poses = []
        self.regions = []
        self.communicator = communicator
 
        self.initUI()
 
 
    def initUI(self):
        self.parent.title("Turtlebot")
        self.style = Style()
        self.style.theme_use("default")
 
        frame = Frame(self, relief=RAISED, borderwidth=1)
        frame.pack(fill=BOTH, expand=1)
 
        self.pack(fill=BOTH, expand=1)
 
        self.label = Label(self, text="No action chosen")
        self.label.pack(side=TOP)
 
        self.defineRegionButton = Button(self, text="Region", command=self.defineRegion)
        self.defineRegionButton.pack(side=RIGHT, padx=5, pady=5)
        self.definePoseButton = Button(self, text="Pose", command=self.definePose)
        self.definePoseButton.pack(side=RIGHT, padx=5, pady=5)
        self.moveButton = Button(self, text="Move", command=self.moveTo)
        self.moveButton.pack(side=RIGHT, padx=5, pady=5)
        self.saveButton = Button(self, text="Save", command=self.save)
        self.saveButton.pack(side=RIGHT, padx=5, pady=5)
        self.loadButton = Button(self, text="Load", command=self.load)
        self.loadButton.pack(side=RIGHT, padx=5, pady=5)
         
        image = Image.open("/tmp/map.pgm")
        self.photo = ImageTk.PhotoImage(image)
 
        self.photoCanvas = Canvas(frame, width=self.photo.width(), height=self.photo.height())
        self.photoCanvas.pack()
        self.redraw()
     
        self.photoCanvas.bind("<Button 1>", self.imageClick)
 
    def defineRegion(self):
        self.defineRegionButton.config(relief=SUNKEN)
        self.definePoseButton.config(relief=RAISED)
        self.moveButton.config(relief=RAISED)
        self.label.config(text="Click on a location on the\nmap to define a region")
        self.state="REGION"
 
    def definePose(self):
        self.defineRegionButton.config(relief=RAISED)
        self.definePoseButton.config(relief=SUNKEN)
        self.moveButton.config(relief=RAISED)
        self.label.config(text="Click on a location on the\nmap to define a pose")
        self.state="POSE"
 
    def moveTo(self):
        self.defineRegionButton.config(relief=RAISED)
        self.definePoseButton.config(relief=RAISED)
        self.moveButton.config(relief=SUNKEN)
        self.label.config(text="Click on a region or a pose\non the map to move Turtlebot")
        self.state="MOVE"
 
    def save(self):
        d = DialogSave(self.parent, "Choose a file name for saving")
        self.parent.wait_window(d.top)
        f = open(d.filename, "w")
        for r in self.regions:
            x, y, n, t, _ = r
            f.write("r " + str(x) + " " + str(y) + " " + str(t) + " " + n + "\n")
        for p in self.poses:
            x, y, n, t, _ = p
            f.write("p " + str(x) + " " + str(y) + " " + str(t) + " " + n + "\n")
            f.close
 
    def load(self):
        d = DialogSave(self.parent, "Type the name of the file to be loaded")
        self.parent.wait_window(d.top)
 
        with open(d.filename, "r") as f:
            for line in f.readlines():
                parts = line.split(' ', 4)
                typ = parts[0]
                x = int(parts[1])
                y = int(parts[2])
                t = float(parts[3])
                n = parts[4]
                print typ, x, y, t, n
                if typ == "r":
                    self.regions.append((x, y, n, t, "region"))
                else:
                    self.poses.append((x, y, n, t, "pose"))
            self.redraw()
 
 
    def imageClick(self, event):
        if self.state == "MOVE":
            closest = self.findClosestPoseOrRegion(event.x, event.y)
            if closest is None:
                return
            x, y, n, r, t = closest
            self.selected = (x, y)
            self.communicator.publishMoveEvent(x / 20.0 - 13.8, y / 20.0 / 13.8)
        elif self.state == "REGION":
            d = DialogRegion(self.parent, "Choose a name and radius for the region")
            self.parent.wait_window(d.top)
            self.regions.append((event.x, event.y, d.text, d.radius, "region"))
        elif self.state == "POSE":
            d = DialogPose(self.parent, "Choose a name for the pose")
            self.parent.wait_window(d.top)
            self.poses.append((event.x, event.y, d.text, d.theta, "pose"))
        elif self.state == "NONE":
            pass
        else:
            print "WARNING: Unknown state:", self.state
 
        self.redraw()
 
    def findClosestPoseOrRegion(self, x, y):
        from sys import maxint
        minsqrdis = maxint
        pose = None
        for p in (self.regions + self.poses):
            dx, dy = (x - p[0], y - p[1])
            sqrdis = dx * dx + dy * dy
            if sqrdis < minsqrdis and sqrdis < 5 * 5:
                minsqrdis = sqrdis
                pose = p
        return pose
             
 
    def redraw(self):
        self.photoCanvas.create_image((3, 3), image=self.photo, anchor="nw")
        if self.selected is not None:
            x, y, = self.selected
            self.photoCanvas.create_oval(x-5, y-5, x+5, y+5, fill="green", outline="green")
        for r in self.regions:
            rad = r[3]*20.0
            self.photoCanvas.create_oval(r[0]-rad,r[1]-rad,r[0]+rad,r[1]+rad,outline="red")
            self.photoCanvas.create_oval(r[0]-2, r[1]-2, r[0]+2, r[1]+2, fill="red", outline="red")
            self.photoCanvas.create_text(r[0], r[1], text=r[2], fill="red", anchor="nw")
        for p in self.poses:
            from math import cos, sin
            the = p[3]
            self.photoCanvas.create_line(p[0], p[1] ,p[0]+15*cos(-the), p[1]+15*sin(-the), fill="blue")
            self.photoCanvas.create_oval(p[0]-2, p[1]-2, p[0]+2, p[1]+2, fill="blue", outline="blue")
            self.photoCanvas.create_text(p[0], p[1], text=p[2], fill="blue", anchor="nw")
 
class Communicator:
    def __init__(self):
        rospy.init_node('x', anonymous=True)
    
        self.pub1 = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub2 = rospy.Publisher('/move_base/goal', PoseStamped, queue_size=1)
        self.pub3 = rospy.Publisher('/move_base/current_goal', PoseStamped, queue_size=1)
    
    def publishMoveEvent(self, x, y):
        print "Published move event", (x, y)

        twist = PoseStamped()
        twist.header.frame_id = 'map'
        twist.pose.position.x = x
        twist.pose.position.y = y
        twist.pose.orientation.z = 0.992267566594
        twist.pose.orientation.w = -0.124117187716

        self.pub1.publish(twist)
        self.pub2.publish(twist)
        self.pub3.publish(twist)

def main():
    root = Tk()
    communicator = Communicator()
    app = MainFrame(root, communicator)
    root.mainloop()
 
if __name__ == '__main__':
    main()
