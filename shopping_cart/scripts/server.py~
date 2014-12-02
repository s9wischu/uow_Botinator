#!/usr/bin/env python

import structures
import threading
import BaseHTTPServer
import json
import math
import os
import urlparse
from mimetypes import types_map
from std_msgs.msg import Bool
from sound_play.msg import SoundRequest
import rospy
from visualization_msgs.msg import Marker
from ar_track_alvar.msg import AlvarMarkers

class Controller: 
    """ This class represents the interface between server and turtlebot. """
    def __init__(self): 
        # Possible states (extensible, but do not change names!): 
        # 'base', 'follow', 'autonomous'
        self.state = "base"   
        # Possible error states (extensible, but do not change names!): 
        # 'None', 'lost-legs'
        self.errorState = "None"
        self.customer = None
        ServerData.pubSound = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)
        self.triggerFollowPublisher = rospy.Publisher('/ours/followModeTrigger', Bool)
        
    # METHODS FOR ACCESS FROM OUTSIDE   
    def onFiducial(self, fiducialNum): 
        print "Read fiducial: " + str(fiducialNum)
        isCustomer = False
        isItem = False
        # Find out if the fiducial belongs to a customer: 
        customer = ServerData.database.getCustomerByFiducialNumber(fiducialNum)
        isCustomer = customer is not None
        
        # Find out if the fiducial belongs to an item: 
        item = ServerData.database.getItemByFiducialNumber(fiducialNum)
        isItem = item is not None
        
        if isCustomer:
            if self.state == "base": 
                if self.customer is None:
                    # Login!
                    print "Customer logged in."
                    print str(customer)
                    ServerData.controller.customer = customer
                    self.onCustomerLoggedIn()
                else:
                    print "Received customer fiducial, but a customer is already logged in."
            elif self.state == "follow" and self.errorState == "lost-legs":
                if customer.getFiducialNum() == self.customer.getFiducialNum(): 
                    # TODO: Reactivate Turtlebot's follow mode. 
                    self.errorState = "None"
                    self.speak("I will now continue following you.")
                    self.triggerFollowPublisher.publish(True)
                else:
                    print "ERROR: tried to reactivate Turtlebot with other customer's card!"
            elif self.state == "autonomous" and self.errorState == "lost-legs":
                #TODO
                pass
            
        elif isItem: 
            if self.state == "follow":
                print "Scanned an item:", item.getName()
                self.speak("Beep!")

                # Check if item price would be beyond budget
                amount = structures.Amount(0, 0)
                amount.add(ServerData.itemList.getTotalAmount())
                amount.add(item.getAmount())
                allergies = ServerData.controller.customer.isAllergic(item)
                if amount.toCents() > ServerData.controller.customer.getAccountBalance().toCents():
                    print "Would exceed account balance!"
                    self.speak("You do not have enough money!")
                    ServerData.messages.append("This would exceed your account balance! Please recharge your account before buying any further items.")
                elif allergies != []:
                    string = reduce(lambda a, b : str(a) + ", " + str(b), allergies)
                    print "Allergic to the following ingredients: " + string + "." 
                    self.speak("You are allegic to an ingredient in this product!")
                    ServerData.messages.append("Warning: You are allergic to the following ingredients: " + string + "." )
                    ServerData.addNewItem(item)
                else: 
                    ServerData.addNewItem(item)
            else:
                print "WARING: Fiducial number", fiducialNum, "belongs to an item, but currently not listening to item fiducials."
            
        else: 
            print "WARNING: Fiducial number", fiducialNum, "belongs to neither",\
                  "item nor customer."                
                              
    def onCustomerLoggedIn(self): 
        # Called when a customer has logged in (but not yet chosen a mode)
        print "Customer '" + self.customer.getName() + "' has logged in."
        self.speak("Welcome, " + self.customer.getName() + "!")

        #ServerData.pubCustomer.publish(self.customer.getName())
                                      
    def onChoseFollowMode(self): 
        # Called when the logged-in customer chooses 'follow' mode
        print "Customer chose 'follow' mode."
        self.speak("I will now follow you.")
        self.state = "follow"
        self.triggerFollowPublisher.publish(True)
    
    def onChoseAutonomousMode(self): 
        # Called when the logged-in customer chooses 'shopping-list' mode
        print "Customer chose 'shopping-list' mode."
        self.state="autonomous"
        # TODO: Start autonomous mode on robot
        pass
        
    def onCheckout(self): 
        # Called when customer chooses to check out. 
        print self.state
        if self.state == "follow":
            self.speak("I now stop following you")
            self.triggerFollowPublisher.publish(False)
        self.triggerFollowPublisher.publish(False)
        print "Customer checked out"
        self.speak("Thank you for shopping with us, " + self.customer.getName() + "!")
        # TODO: Move back to base position. 
        self.customer = None
        
    def onLegsLost(self): 
        # Called when robot loses track of legs
        print "Turtlebot lost track of legs."
        self.speak("I lost track of you! Please reactivate me by holding your customer card in front of the webcam.")
        # TODO: Call this method when Turtlebot lost track of legs
        # TODO: Acoustic warning. 
        self.errorState = "lost-legs"
        
    def onReturnedToBase(self): 
        # Called when turtlebot returned to base. 
        print "Turtlebot returned to base."
        # TODO: Call this method when Turtlebot returned to base. 
        self.state = "base"
        
    def onGuideRequest(self, item):
        # Called when customer asked to be guided to given item. 
        self.speak("You will now be guided to the item you selected. Please follow me.")
        goal = item.getLocation()
        print "Customer asks to be guided to '" + item.getName() + \
              "' at location " + str(goal) + "."
        # TODO: implement    
        
    def speak(self, text):
        soundRequest = SoundRequest()
        soundRequest.sound = -3 #Say text
        soundRequest.command = 1 #Play once 
        soundRequest.arg = text
        ServerData.pubSound.publish(soundRequest)


class ServerData:
    """ Static class that stores information for server. Note that
        this server copy with only one client at a atime. """
        
        
    # List of items that have been added to the cart but are not yet displayed
    # in the list of items at the client. They will be sent on the next update. 
    newItems = []
    
    # Items in the cart
    itemList = structures.ItemList()   
    
    # Database with list of customers and items
    database = structures.Database.fromXMLFile("/home/motionlab/catkin_ws/src/shopping_cart/scripts/database.xml")
    
    # This value is set whenever the customer selects an item in the list. 
    # It will be sent to the client on the next update and displayed on the 
    # right-hand side of the window. 
    newItem = None

    # Lock used for mutual exclusion
    lock = threading.Lock()
    
    # Has the server been restarted? 
    restarted = True
    
    controller = None
    
    messages = []
    
    @staticmethod 
    def getNewItems(): 
        """ Items that were added to the cart since the client's last update """
        ServerData.lock.acquire()
        result = list(ServerData.newItems)
        ServerData.newItems = []
        ServerData.lock.release()
        return result
   
    @staticmethod
    def addNewItem(item): 
        """ Adds a new item to the cart """
        ServerData.lock.acquire()
        ServerData.newItems.append(item)
        ServerData.itemList.addItem(item)
        # If this is the first item, select it
        ServerData.lock.release()
        if len(ServerData.itemList.getItems()) == 1: 
            ServerData.changeItem(item)
        
        
    @staticmethod
    def changeItem(item): 
        """ Change the item that is highlighted on the client and shown on the 
            right-hand side of the window """
        if item is None: 
            print "WARNING: Change item to None!"
        ServerData.lock.acquire()
        ServerData.newItem = item
        ServerData.lock.release()
        
    @staticmethod
    def getItem(): 
        """ Get the item that is currently highlighted. Returns None if the 
            item has not changed since the last call """
        ServerData.lock.acquire()
        product = ServerData.newItem
        ServerData.newItem = None
        ServerData.lock.release()
        return product
        
    
        
        
     
class CallbackHTTPServer(BaseHTTPServer.HTTPServer):
    
    def server_activate(self): 
        self.RequestHandlerClass.pre_start()
        HTTPServer.server_activate(self)
        self.RequestHandlerClass.post_start()
    
    def server_close(self): 
        self.RequestHandlerClass.pre_start()
        HTTPServer.server_close(self)
        self.RequestHandlerClass.post_stop()

class Server: 
    PORT = 31415        

    def __init__(self):
        self.keepRunning = True
        self.running = False
                
    def serve(self):
        self.running = True
        server_address = ("", self.PORT)
        self.httpd = BaseHTTPServer.HTTPServer(server_address, RequestHandler)
        self.httpd.timeout = 1
        print "Started server at %s:%s" % (self.httpd.server_name, self.httpd.server_port)
        while self.keepRunning: 
            self.httpd.handle_request()
        print "Closing server ...", 
        self.httpd.server_close(); 
        print "Closed."
        

    def start(self): 
        ServerData.restarted = True
        self.thread = threading.Thread(target=self.serve)
        self.thread.deamon = True
        self.thread.start()
        
    def stop(self):
        self.keepRunning = False
        self.thread.join()

    def isRunning(self): 
        return self.running

class RequestHandler(BaseHTTPServer.BaseHTTPRequestHandler):

    def log_message(self, format, *args):
        return
      
    @classmethod
    def pre_start(cls):
        pass
        
    @classmethod
    def post_start(cls):
        pass
        
    @classmethod
    def pre_stop(cls):
        pass
        
    @classmethod
    def post_stop(cls): 
        pass
    
    def do_GET(self):
        """ handle get requests """
        query_string = urlparse.urlparse(self.path).query
        parameters = urlparse.parse_qs(query_string)
        
        if "type" in parameters: 
            # This request is a command
            type = parameters["type"][0]
            if type == "update":
                self.sendNewItems()   
                return
            elif type == "item-clicked":
                # Find an item with the same fiducial number in the list:
                fiducialNum = int(float(parameters["itemClicked"][0]))
                
                item = ServerData.database.getItemByFiducialNumber(fiducialNum)
                
                if item == None: 
                    print "WARNING: Item with fiducial number ", str(fiducialNum), "could not be found"
                    return
                    
                ServerData.changeItem(item);         
                return
            elif type == "remove-item":
                fiducialNum = int(float(parameters["remove_id"][0]))
                # Find first item with the corresponding fiducial number: 
                deleted = False
                for item in ServerData.itemList.getItems():
                    if item.getFiducialNum() == fiducialNum:
                        ServerData.itemList.removeById(item.getId())
                        ServerData.newItems = ServerData.itemList.getItems()
                        ServerData.newItem = structures.Item()
                        ServerData.restarted = True
                        deleted = True
                        break; 
                        
                print len(ServerData.newItems)
                if not deleted: 
                    print "WARNING: Item with fiducial number ", str(fiducialNum), "could not be found"
                    
                print "Deleted item with fiducial number", fiducialNum
                return 
                
            elif type == "get-item":
                fiducialnum = int(float(parameters["fiducial"][0]))
                print "Requested product information about product with fiducial number ", fiducialnum
                item = ServerData.database.getItemByFiducialNumber(fiducialnum)
                if item is None: 
                    print "WARNING: Item with fiducial number ", str(fiducialNum), "could not be found"
                    return 
                
                structure = {"item" : item.getStructure()}
                self.send_response(200)
                self.send_header("Content-type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps(structure)); 
                return
                
            elif type == "search":
                if "search_string" in parameters.keys(): 
                    searchString = parameters["search_string"][0]
                    searchResult = ServerData.database.search(searchString)
                    structure = {"items" : map(lambda x : x.getHTMLListString(), searchResult)}
                    print "Search request for \"", searchString, "\"."
                else: 
                    structure = {"items" : [] }
                    print "Search request for \"\"."
                    
                self.send_response(200)
                self.send_header("Content-type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps(structure)); 
                return
            
            elif type == "has-customer": 
                self.sendStructure({"hasCustomer" : ServerData.controller.customer is not None})
                return
                
            elif type == "chose-follow": 
                if ServerData.controller.customer is None: 
                    print "WARNING: Tried to enter follow mode, but no customer logged in!"
                    self.sendStructure({"ready" : False})
                    return
                ServerData.controller.onChoseFollowMode()
                self.sendStructure({"ready" : True})
                return
            
            elif type == "get-customer-data": 
                if ServerData.controller.customer is None: 
                    self.sendStructure(None)
                else: 
                    self.sendStructure(ServerData.controller.customer.getStructure())
                return
                
            elif type == "checkout":             
                totalAmount = ServerData.itemList.getTotalAmount()
                oldBalance = ServerData.controller.customer.getAccountBalance()
                newBalance = structures.Amount(0, 0)
                newBalance.fromCents(oldBalance.toCents() - totalAmount.toCents())
                self.sendStructure({"amount" : str(totalAmount), "balance" : str(newBalance)})
                ServerData.controller.onCheckout()
                return
                
            elif type == "is-base":
                self.sendStructure({"isBase" : ServerData.controller.state == "base"})
                return
                
            elif type == "error-state":
                self.sendStructure({"errorState" : ServerData.controller.errorState, \
                                    "state" : ServerData.controller.state})
                return
            
            elif type == "guide-to":
                fiducialNum = int(float(parameters["fiducial"][0]))
                item = ServerData.database.getItemByFiducialNumber(fiducialNum)
                ServerData.controller.onGuideRequest(item)
                self.sendStructure({})
                return

        else:
            # This is a request for a page. 
            try:
                # Get file path (without GET string). Also remove trailing slash
                path = urlparse.urlparse(self.path).path[1:]
                path = "/home/motionlab/catkin_ws/src/shopping_cart/scripts/" + path
                # Extract file ending
                _, ext = os.path.splitext(path)
                # Extract content type
                contentType = types_map[ext]
                
                if path == "/home/motionlab/catkin_ws/src/shopping_cart/scripts/html/product_list.html":
                    # When the page is reloaded, add all items to the "new" list
                    # such that they are reloaded. 
                    items = ServerData.itemList.getItems()
                    ServerData.newItems = items
                    if len(items) > 0:
                        ServerData.changeItem(items[0]);
                
                print os.getcwd()
                
                # Open file and send response
                with open(path) as f:
                    self.send_response(200)
                    self.send_header('Content-type', contentType)
                    self.end_headers()
                    self.wfile.write(f.read())
                    
            except IOError:
                self.send_error(404)
            
            return

        # response "no content"
        self.send_response(204)
        return

    def sendNewItems(self):
        # Sends items that have been added since last "update" request. 
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.end_headers()
        
        structure = {}
        
        # Add the new items
        newItems = ServerData.getNewItems()
        structure["numberNew"] = len(newItems)
        structure["html"] = []
        for newItem in newItems: 
            structure["html"].append(newItem.getHTMLListString())
        
        # Check if the highlighted item has changed
        highlightedItem = ServerData.getItem()
        if highlightedItem is not None: 
            print "Highlighted product changed: ", highlightedItem.getName()
            structure["hasNewHighlighted"] = True
            structure["item"] = highlightedItem.getStructure()
        else: 
            structure["hasNewHighlighted"] = False
            
        structure["messages"] = ServerData.messages
        ServerData.messages = []
        
        
        if ServerData.restarted: 
            # Set the 6highlighted item to the first item, or show a dummy item
            # (when no items) when restarted
            structure["hasNewHighlighted"] = True
            items = ServerData.itemList.getItems()
            if len(items) > 0: 
                structure["item"] = items[0].getStructure()
            else:
                structure["item"] = structures.Item().getStructure()
                
        
        # Add information if server has been restarted
        structure["restarted"] = ServerData.restarted
        ServerData.restarted = False
            
        # Total amount
        structure["total"] = str(ServerData.itemList.getTotalAmount())
        
        structure["hasCustomer"] = ServerData.controller.customer is not None
        
        structure["errorState"] = ServerData.controller.errorState
        
        self.wfile.write(json.dumps(structure)); 
        return 
     

    def sendStructure(self, structure): 
        """ Sends a disctionary as JSON """
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(structure)); 
     
     
def fiducialEvent(scan):
    global lastFiducialTime
    from time import time
    
    TIME_BETWEEN_SCANS = 5.0
    
    if len(scan.markers) == 0:
        return
    
    # Make sure that fiducials are not registered too often
    if time() - lastFiducialTime > TIME_BETWEEN_SCANS and scan.markers[0].id != 0 and scan.markers[0] != 255:
        ServerData.controller.onFiducial(scan.markers[0].id)
        lastFiducialTime = time()
     
if __name__ == '__main__':
    global lastFiducialTime
    rospy.init_node('server', anonymous=True)                      

    ServerData.controller = Controller()

    
    lastFiducialTime = 0.0
    
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, fiducialEvent)
    
    server = Server()
    server.serve()
#if __name__ == '__main__':
#
#    server = Server()
#    server.start()
#   
#    ServerData.controller = Controller()
#    
#    try:
#        
#        print "q  : terminate server"
#        
#        print "Trigger Turtlebot events with the following commands:"
#        print "f #: read fiducial with given number"
#        print "l  : leg error"
#        print "b  : returned to base"
#        
#        while True: 
#            code = raw_input()
#            codes = code.split(" ")
#            if codes[0] == "q": 
#                break
#            elif codes[0] == "f":
#                fiducial = int(float(codes[1]))
#                ServerData.controller.onFiducial(fiducial)
#            elif codes[0] == "l": 
#                ServerData.controller.onLegsLost()
#            elif codes[0] == "b": 
#                ServerData.controller.onReturnedToBase()
#   
#    except Exception as e:
#        print e
#    finally:
#        server.stop()
                
