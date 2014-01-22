#!/usr/bin/env python
#TODO: Delete unused imports
import logging
import rospy
import threading
import time
 
import cflib.crtp
from cfclient.utils.logconfigreader import LogConfig
from cfclient.utils.logconfigreader import LogVariable
from cflib.crazyflie import Crazyflie
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Float32
from std_msgs.msg import String
 
logging.basicConfig(level=logging.DEBUG)

class CrazyflieURI:
    """
    Class for finding crazyflies and saving their connection-information.
    It will allow to find, add, find and add, print and modify all found
    information and connection-data.
    Partially the execution will be interactive/ require interaction.
    """
    def __init__(self):
        """
        Creating and empty list for uri connection-data.
        The amount of found links is set to zero.
        """
        self.uri_list = []
        self.amount = 0

    def add(self, uri, name):
        """
        URI and name are saved. Since no name was found so far, this method
        will not be implemented for the time being...
        """
        pass

    def add_uri(self, uri):
        """
        URI is stored in list (the list is appended) and the variable amount
        is incremented
        """
        self.amount += 1
        self.uri_list.append(uri)
        self.uri_list.append(name)
    
    def find_all(self):
        """
        This method will list all found connections or links and their
        names.
        Adding a specific link will be interactive.
        """
        #TODO: specify above

        print("Type in:")
        print("b - blink, y - add to list, n (else) - do not add to list")
        cflib.crtp.init_drivers()
        available = cflib.crtp.scan_interfaces()
        for i in available:
            print "InterfacewithURI [%s] found, name [%s]" % (i[0],i[1])
            ch = sys.stdin.read(1)
            if ch == "b":
                self.crazyflie.open_link(i[0])
                #TODO: Add blink-method, extra class?
                self.crazyflie.close_link()
            elif ch == "y":
                self.add_uri(i[0])
                print"Added [%s]" % (i[0])
            else:
                print "Not added"

        if self.amount is 0:
            print "No link found"
        print("end") 

    def find_add_all(self):
        """
        This method will list all found connections or links and their
        names and add all URIs to the uri-list.
        Adding a specific link will not be possible here.
        """
        print("start find_add_all")
        cflib.crtp.init_drivers()
        available = cflib.crtp.scan_interfaces()
        for i in available:
            print "InterfacewithURI [%s] found, name [%s]" % (i[0],i[1])
            self.add_uri(i[0])
        print("end") 

    def print_all(self):
        """
        Print all links stored in the list of URIs
        """
        print "Saved in list of URIs:"
        for i in xrange(self.amount):
            print "[%s] : [%s]" % (i, self.uri_list[i])
        print "end"

class Blink:
    def blink(req):
        #TODO

    def blink_server():
        rospy.init_node('blink')
        s = rospy.Service('blink', crazyflie.srv.Blink, blink)
        rospy.spin()



