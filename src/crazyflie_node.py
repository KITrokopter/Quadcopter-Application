#!/usr/bin/env python
#
# crazyflie_node.py
#
# Copyright (c) 2013 Carina Kuebler, Dominik Kiefer, Dennis Keck 
# 	based on crazyflie_node.py by Jesse Rosalia
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import rospy
import logging
import roslib
roslib.load_manifest('quadcopter_application')
import cflib.crtp
import std_msgs.msg
from cflib.crazyflie.log import Log, LogVariable, LogConfig
from cflib.crazyflie import Crazyflie
#from quadcopter_application.msg import * quadcopter_controll
from quadcopter_application.srv import *
from quadcopter_application.msg import * 

logging.basicConfig(level=logging.DEBUG)

#class for the cf vars which shall be used
class LogVar(object):
  def __init__(self, var, vartype='float'):
    self.var = var
    self.vartype = vartype

LOGVARS = [
    LogVar('stabilizer.roll', 'float'),
    LogVar('stabilizer.pitch', 'float'),
    LogVar('stabilizer.yaw', 'float'),
    LogVar('stabilizer.thrust', 'uint16_t'),
    LogVar('altimeter.pressure'),
    LogVar('mag.x', 'int16_t'),
    LogVar('mag.y', 'int16_t'),
    LogVar('mag.z', 'int16_t'),
    LogVar('acc.x', 'float'),
    LogVar('acc.y', 'float'),
    LogVar('acc.z', 'float'),
    LogVar('gyro.x', 'float'),
    LogVar('gyro.y', 'float'),
    LogVar('gyro.z', 'float'),
    LogVar('motor.m1', 'uint16_t'),
    LogVar('motor.m2', 'uint16_t'),
    LogVar('motor.m3', 'uint16_t'),
    LogVar('motor.m4', 'uint16_t'),
    LogVar('pm.vbat', 'float')
]



class CrazyflieNode:
    """
    Class is required so that methods can access the object fields.
    """
    def __init__(self):
	print("init started")
        """
        Connect to Crazyflie, initialize drivers and set up callback.
 
        The callback takes care of logging the accelerometer, altimeter, barometer, gyrometer and magnetometer values.
        """
        # TODO Get quadcopter id - what format has the id?!?
        self.id = 0
        self.link_status = "Unknown"
        self.link_quality = 0.0
        self.battery_status = 0.0
        self.packetsSinceConnection = 0
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.motor_m1 = 0
        self.motor_m2 = 0
        self.motor_m3 = 0
        self.motor_m4 = 0
        
        self.altimeter = 0.0 #TODO: Why do we set these values?
        
        self.mag_x = 0.0
        self.mag_y = 0.0
        self.mag_z = 0.0
        
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
        
        self.cmd_thrust = 0
        self.cmd_pitch = 0.0
        self.cmd_roll = 0.0
        self.cmd_yaw = 0.0
        
        #TODO: Call of function in run(), __init__, ... ?
        #open_link_server()
        #close_link_server()

        # Init the callbacks for the crazyflie lib
        self.crazyflie = Crazyflie()
        cflib.crtp.init_drivers()

        # Init the status publisher topic for ROS
	# TODO Talk about new implemenatation
        self.publisher  = rospy.Publisher('quadcopter_status_' + str(self.id), quadcopter_status, latch=True)

	# TODO Which callbacks are still needed?
        # Connection callbacks
        self.crazyflie.connectionInitiated.add_callback(self.connectionInitiated)
        self.crazyflie.connectSetupFinished.add_callback(self.connectSetupFinished)
        self.crazyflie.connected.add_callback(self.connected)
        self.crazyflie.disconnected.add_callback(self.disconnected)
        self.crazyflie.connectionLost.add_callback(self.connectionLost)
        self.crazyflie.connectionFailed.add_callback(self.connectionFailed)

        # Link quality callbacks
        self.crazyflie.linkQuality.add_callback(self.linkQuality)
        self.crazyflie.receivedPacket.add_callback(self.receivedPacket)
        self.crazyflie.open_link("radio://0/11/250K")
	
	rospy.Subscriber('quadcopter_controll_' + str(self.id), quadcopter_controll, self.set_controll)

    def handle_open_link(req):
        #self.crazyflie.open_link("radio://0/11/250K")
        self.crazyflie.open_link(req.string_uri)

    def open_link_server():
        rospy.init_node('open_link')
        s = rospy.Service('open_link', open_link, handle_open_link)
        print "Ready to open a link to a quadcopter."
        rospy.spin()    # Necessary? Working? Enough?
	
    def handle_close_link(req):
        shut_down()
        #TODO: match this function to srv-file

    def close_link_server():
        rospy.init_node('close_link')
        s = rospy.Service('close_link', close_link, handle_close_link)
        print "Ready to close a link to a quadcopter."
        rospy.spin()    # Necessary? Working? Enough?
	
        #TODO: Functionality? (Dominik)
        #TODO: Test Start 

    def handle_blink(req):
        pass

    def blink_server():
        rospy.init_node('blink')
        s = rospy.Service('blink', blink, handle_blink)
        print "Ready to blink."     #TODO: Check if link exists.
        rospy.spin()    # Necessary? Working? Enough?

    def start(self):
        thrust_mult = 1
        thrust_step = 500
        thrust = 20000
        pitch = 0
        roll = 0
        yawrate = 0
        while thrust >= 20000:
            self.crazyflie.commander.send_setpoint(roll, pitch, yawrate, thrust)
            time.sleep(0.1)
            if (thrust >= 25000):
                thrust_mult = -1
            thrust = thrust + (thrust_step * thrust_mult)
        self.crazyflie.commander.send_setpoint(0,0,0,0)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
        self.crazyflie.close_link()

    def shut_down(self):
        try:
	    print("shut down")
            #self.pitch_log.stop()
        finally:
            self.crazyflie.close_link()

    def connectionInitiated(self, linkURI):
        self.link_status = "Connection Initiated"

    def connectSetupFinished(self, linkURI):
        print("Connection Setup finished") #DEBUG
        self.link_status = "Connect Setup Finished"
        
        """
        Configure the logger and start recording.
 
        The logging variables are added one after another to the logging
        configuration. Then the configuration is used to create a log packet
        which is cached on the Crazyflie. If the log packet is None, the
        program exits. Otherwise the logging packet receives a callback when
        it receives data, which prints the data from the logging packet's
        data dictionary as logging info.
        """
        print("start logs")
        lg = LogConfig("Stabalizer", 100)
	for f in LOGVARS:
	    lg.add_variable(f.var)
	print("added vars to log")
	self.crazyflie.log.add_config(lg)
	print("added conf")
	if lg.valid:
	    print("valid")
	    lg.data_received_cb.add_callback(self.onLogData)
            print("added data cb")
            lg.error_cb.add_callback(self.onLogError)
            print("added error cb")
            lg.start()
            print("started log")
	else:
	    print("invalid")
            logger.warning("Could not setup logconfiguration after connection!")
    
    def onLogError(self, data):
	logger.warning("Log error!")
    
    def onLogData(self, data):
	print("got data from cf")
	# DEBUG
        rospy.loginfo("Accelerometer: x=%.2f, y=%.2f, z=%.2f" %
                        (data["acc.x"], data["acc.y"], data["acc.z"]))
	
	self.altimeter = data['altimeter.pressure']
	self.battery_status = data['pm.vbat']
	self.mag_x = data['mag.x']
	self.mag_y = data['mag.y']
	self.mag_z = data['mag.z']
	
	self.roll = data['stabilizer.roll']
	self.pitch = data['stabilizer.pitch']
	self.yaw = data['stabilizer.yaw']
	
	self.acc_x = data['acc.x']
	self.acc_y = data['acc.y']
	self.acc_z = data['acc.z']
	
	self.gyro_x = data['gyro.x']
	self.gyro_y = data['gyro.y']
	self.gyro_z = data['gyro.z']
	
	self.motor_m1 = data['motor.m1']
	self.motor_m1 = data['motor.m2']
	self.motor_m1 = data['motor.m3']
	self.motor_m1 = data['motor.m4']

    def connected(self, linkURI):
        self.packetsSinceConnection = 0
        self.link_status = "Connected"
        print("Connected") #DEBUG

    def disconnected(self, linkURI):
        self.link_status = "Disconnected"
     
    def connectionLost(self, linkURI, errmsg):
        self.link_status = "Connection Lost - " + errmsg
 
    def connectionFailed(self, linkURI, errmsg):
        self.link_status = "Connection Failed - " + errmsg
 
    def linkQuality(self, percentage):
        self.link_quality = percentage

    def batteryStatus(self, percentage):
        self.battery_status = percentage

    def receivedPacket(self, pk):
        self.packetsSinceConnection += 1

    def log_pitch_data(self, data):
        self.pitch  = data["stabilizer.pitch"]
        self.roll   = data["stabilizer.roll"]
        self.thrust = data["stabilizer.thrust"]
        self.yaw    = data["stabilizer.yaw"]

    def set_controll(self, data):
        rospy.loginfo(rospy.get_name() + ": Setting thrust to: %d" % data.thrust)
        self.cmd_thrust = data.thrust

    def run_node(self):
	#h = std_msgs.msg.Header()
	#h.stamp = rospy.Time.now

        msg = quadcopter_status()
	msg.header = h
	msg.id = self.id
	msg.battery_status = self.battery_status
	msg.link_quality = self.link_quality
	msg.altimeter = self.altimeter
	msg.mag_x = self.mag_x
	msg.mag_y = self.mag_y
	msg.mag_z = self.mag_z

	msg.gyro_x = self.gyro_x
	msg.gyro_y = self.gyro_y
	msg.gyro_z = self.gyro_z

	msg.acc_x = self.acc_x
	msg.acc_y = self.acc_y
	msg.acc_z = self.acc_z

	msg.motor_m1 = self.motor_m1
	msg.motor_m2 = self.motor_m2
	msg.motor_m3 = self.motor_m3
	msg.motor_m4 = self.motor_m4

	msg.roll = self.roll
	msg.pitch = self.pitch
	msg.yaw = self.yaw

        self.publisher.publish(msg)
        # Send commands to the Crazyflie
        # DEBUG
	#rospy.loginfo(rospy.get_name() + ": Sending setpoint: %f, %f, %f, %d" %
	#       (self.cmd_roll, self.cmd_pitch, self.cmd_yaw, self.cmd_thrust))
        self.crazyflie.commander.send_setpoint(self.cmd_roll, self.cmd_pitch, self.cmd_yaw, self.cmd_thrust)
         
def run():
    #TODO: understand the following comment
    # Init the ROS node here, so we can split functionality
    # for this node across multiple classes        
    rospy.init_node('crazyflie')
    print("ros node initialized")

    #TODO: organize this into several classes that monitor/control one specific thing (if necessary)
    node = CrazyflieNode()
    print("cf node initialized")
    while not rospy.is_shutdown():
        node.run_node()
        rospy.sleep(0.1)
    node.shut_down()
        
        
if __name__ == '__main__':
    run()
