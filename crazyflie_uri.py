#!/usr/bin/env python
#TODO: Delete unused imports
#TODO: Comment
import logging
import rospy
import threading
import time
 
import cflib.crtp
from cfclient.utils.logconfigreader import LogConfig
from cfclient.utils.logconfigreader import LogVariable
from cflib.crazyflie import Crazyflie
#from cflib import crazyflie
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Float32
from std_msgs.msg import String
 
logging.basicConfig(level=logging.DEBUG)

class CrazyflieURI:
    def __init__(self):
        self.uri_list = []
        self.amount = 0

    def add(self, uri, name):
        self.amount += 1
        self.uri_list.append(uri)
        self.uri_list.append(name)
    
    def find_all(self):
        print("start")
        cflib.crtp.init_drivers()
        available = cflib.crtp.scan_interfaces()
        for i in available:
            self.add(i[0], i[1])
            print "InterfacewithURI [%s] found, name [%s]" % (i[0],i[1])
        print("end") 

    def find_add_all(self):
        pass

    def print_all(self):
        print "start print_all"
        for i in xrange(self.amount):
            print "Found: [%s]" % (self.uri_list[i])
        print "end"


