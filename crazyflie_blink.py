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

class CrazyflieBlink:
    """
    Class for blinking
    """
    def __init__(self):
        """
        """
        pass

    def blink(self, uri):
        """
        Try to connect to given URI, blink and disconnect.
        """
        cflib.crtp.init_drivers()
        self.crazyflie.open_link(uri)
        #TODO: Add blink-method, extra class?
        self.crazyflie.close_link()
        


