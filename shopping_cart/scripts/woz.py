#!/usr/bin/env python

# for kinect image stream
# web_video_server package

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('trajectory_msgs')


import rospy
import threading
from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import (
    Twist,
    Vector3,
    Point
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint
)
from sound_play.msg import SoundRequest

#from sound_play.libsoundplay import SoundClient



import BaseHTTPServer
import json
import math
import os
import urlparse
from mimetypes import types_map


mobile_base_velocity = None
sound_client = None

class RequestHandler(BaseHTTPServer.BaseHTTPRequestHandler):
    def do_GET(self):
        global publisher
        query_string = urlparse.urlparse(self.path).query
        parameters = urlparse.parse_qs(query_string)

        if 'type' not in parameters:
            try:
                if self.path == "/":
                    self.path = "/index.html"
                elif self.path == "favicon.ico":
                    return
                elif self.path == "map.gif":
                    # Draw robot position on map and send image back
                    draw_map()
                    return
                fname,ext = os.path.splitext(self.path)
                print "Loading file", self.path
                with open(os.path.join(os.getcwd(),self.path[1:])) as f:
                    self.send_response(200)
                    self.send_header('Content-type', types_map[ext])
                    self.end_headers()
                    self.wfile.write(f.read())
                return
            except IOError:
                self.send_error(404)
                return

        command_type = parameters['type'][0]
        
        if command_type == 'base':
            base_x = 0.0
            if 'x' in parameters:
                base_x = float(parameters['x'][0])
            base_y = 0.0
            if 'y' in parameters:
                base_y = float(parameters['y'][0])
            base_z = 0.0
            if 'z' in parameters:
                base_z = float(parameters['z'][0])
            twist_msg = Twist()
            twist_msg.linear = Vector3(base_x, base_y, 0.0)
            twist_msg.angular = Vector3(0.0, 0.0, base_z)
            mobile_base_velocity.publish(twist_msg)

        elif command_type == 'speak':
            text = parameters['say'][0]
            sr = SoundRequest()
            sr.sound = -3 #Say text
            sr.command = 1 #Play once
            sr.arg = text
            publisher.publish(sr)
            #os.system("espeak -s 155 -a 200 '" + text + "' ")


        # response
        self.send_response(204)
        return

    def log_message(self, format, *args):

        return

    def return_map():
        # Read blank_map and save to map
        pass




def initialize_robot():
    global mobile_base_velocity
    mobile_base_velocity = rospy.Publisher('/mobile_base/commands/velocity', 
                                           Twist, queue_size=10)

if __name__ == '__main__':
    global publisher
    server_address = ('', 31416)
    httpd = BaseHTTPServer.HTTPServer(server_address, RequestHandler)
    #httpd.timeout = 5  # set timeout to 5 seconds to check for shutdown
    

    rospy.init_node('http_server_node', anonymous=True)
    
    # initialize action clients and publishers
    # mobile_base_velocity = rospy.Publisher('/base_controller/command', Twist)

    initialize_robot()
    #sound_client = SoundClient()

    print "HTTP server started at {}:{}".format(httpd.server_name, 
                                                httpd.server_port)
    
    publisher= rospy.Publisher('/robotsound', SoundRequest, queue_size=10)

    # spin
    while not rospy.is_shutdown():
        httpd.handle_request()


    print "\nHTTP server stopped"
