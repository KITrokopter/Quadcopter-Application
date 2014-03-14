#!/usr/bin/env python2.7
import rospy
import time
import threading
import logging

from control_application.msg import quadcopter_movement

logger = logging.getLogger()

ID = 0
INITIAL_ROLL = 0.0
INITIAL_PITCH = 0.0
INITIAL_YAW = 0.0
INITIAL_THRUST = 20001

class CrazyDemo(object):

    def start(self):
        import sys
        factor = 1000
        thrust = INITIAL_THRUST
        roll = INITIAL_ROLL
        pitch = INITIAL_PITCH
        yaw = INITIAL_YAW
        pub = rospy.Publisher('quadcopter_movement_' + str(ID), quadcopter_movement)
        rospy.init_node('crazyflie_sender')
        
        msg = quadcopter_movement()
        msg.header.stamp = rospy.Time.now()
        msg.thrust = 40000
        msg.roll = roll
        msg.pitch = pitch
        msg.yaw = yaw
        pub.publish(msg)
        rospy.sleep(2)
        msg = quadcopter_movement()
        msg.header.stamp = rospy.Time.now()
        thrust = 39000
        msg.thrust = 39000
        msg.roll = roll
        msg.pitch = pitch
        msg.yaw = yaw
        pub.publish(msg)

        while not rospy.is_shutdown():
            ch = sys.stdin.read(1)    
            if ch == "a":
                thrust = thrust + factor
            elif ch == "q":
                thrust = thrust + 10 * factor    
            elif ch == "y":
                thrust = thrust - factor
            elif ch == "s":
                roll = roll + 1.0
            elif ch == "x":
                roll = roll - 1.0
            elif ch == "d":
                pitch = pitch + 1.0
            elif ch == "c":
                pitch = pitch - 1.0 
            elif ch == "f":
                yaw = yaw + 1.0
            elif ch == "v":
                yaw = yaw - 1.0  
            elif ch == "e":
                thrust = 0
            else:
                thrust = thrust;
            rospy.loginfo(str)
            msg = quadcopter_movement()
            msg.header.stamp = rospy.Time.now()
            msg.thrust = thrust
            msg.roll = roll
            msg.pitch = pitch
            msg.yaw = yaw
            pub.publish(msg)
            rospy.sleep(0.05)

c = CrazyDemo()
c.start()
time.sleep(1)
c.stop()


