#!/usr/bin/env python2.7
import rospy
import time
import threading
import logging

from std_msgs.msg import UInt16

logger = logging.getLogger()

INITIAL_ROLL = 0.0
INITIAL_PITCH = 0.0
INITIAL_YAWRATE = 0
INITIAL_THRUST = 10001

class CrazyDemo(object):

    def start(self):
        import sys
        range = 350
        factor = 1000
        thrust = INITIAL_THRUST
	pub = rospy.Publisher('thrust', UInt16)
	rospy.init_node('crazyflie')
        thrust = thrust + 20000
        while not rospy.is_shutdown():
            ch = sys.stdin.read(1)    
            if ch == "u":
                thrust = thrust + factor
            elif ch == "e":
                thrust = 0
            elif ch == "d":
                thrust = thrust - factor
            rospy.loginfo(str)
	    pub.publish(thrust)
	    rospy.sleep(0.05)



c = CrazyDemo()
c.start()
time.sleep(1)
c.stop()


