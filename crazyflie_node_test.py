#!/usr/bin/env python
#
# crazyflie_node.py
#
# Copyright (c) 2013 Jesse Rosalia
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

from crazyflie_uri import CrazyflieURI 

logging.basicConfig(level=logging.DEBUG)

class CrazyflieNode:
    """
    Class is required so that methods can access the object fields.
    """
    def __init__(self):
        """
        Connect to Crazyflie, initialize drivers and set up callback.
 
        The callback takes care of logging the accelerometer values.
        """
        self.link_status = "Unknown"
        self.link_quality = 0.0
        self.packetsSinceConnection = 0
        self.motor_status = ""
        self.pitch = 0.0
        self.roll = 0.0
        self.thrust = 0
        self.yaw = 0.0

        self.cmd_thrust = 0
        self.cmd_pitch = 0.0
        self.cmd_roll = 0.0
        self.cmd_yaw = 0.0
        
        # Array for storing found URIs
        self.uri_list = CrazyflieURI()
        self.uri_list.find_all()
        self.uri_list.print_all()

        # Init the callbacks for the crazyflie lib
        self.crazyflie = Crazyflie()
        cflib.crtp.init_drivers()

        # Init the published topics for ROS, for this class
        self.link_status_pub  = rospy.Publisher('link_status', String, latch=True)
        self.link_quality_pub = rospy.Publisher('link_quality', Float32)
        self.packet_count_pub = rospy.Publisher('packet_count', UInt32)

        #self.battery_status_pub = rospy.Publisher('battery_status', Float32)
        #self.packet_count_pub = rospy.Publisher('packet_count', UInt32)

        self.motor_status_pub = rospy.Publisher('motors', String)

        self.pitch_pub        = rospy.Publisher('stabilizer/pitch', Float32)
        self.roll_pub         = rospy.Publisher('stabilizer/roll', Float32)
        self.thrust_pub       = rospy.Publisher('stabilizer/thrust', Float32)
        self.yaw_pub          = rospy.Publisher('stabilizer/yaw', Float32)
 
        rospy.Subscriber('thrust', UInt16, self.set_thrust)
        #rospy.Subscriber('pitch', float, self.set_pitch)
        #rospy.Subscriber('roll', float, self.set_roll)
        #rospy.Subscriber('yaw', float, self.set_yaw)

        # Connection callbacks
        #TODO: for a lot of these, we just update the status and/or publish a value
        # it would be sweet if we could create a generic function to do that for us,
        # instead of using all of these callbacks.
        self.crazyflie.connectionInitiated.add_callback(self.connectionInitiated)
        self.crazyflie.connectSetupFinished.add_callback(self.connectSetupFinished)
        self.crazyflie.connected.add_callback(self.connected)
        self.crazyflie.disconnected.add_callback(self.disconnected)
        self.crazyflie.connectionLost.add_callback(self.connectionLost)
        self.crazyflie.connectionFailed.add_callback(self.connectionFailed)

        # Link quality callbacks
        self.crazyflie.linkQuality.add_callback(self.linkQuality)
        self.crazyflie.receivedPacket.add_callback(self.receivedPacket)
        
        #TODO: should be configurable, and support multiple devices
        self.crazyflie.open_link("radio://0/10/250K")

    def find_crazyflies(self):
        print("start")
        available = cflib.crtp.scan_interfaces()
        for i in available:
                print "InterfacewithURI [%s] found, name [%s]" % (i[0],i[1])
        print("end") 
        
    def node_blink(self, string_link):
        rospy.wait_for_service('blink')
        blink_service = rospy.ServiceProxy('blink', crazyflie.srv.Blink)
        try:
            response = blink(string_link)
        except:
            print("Service did not process request: " + str(exc))
        #TODO: ...

    def shut_down(self):
        try:
            self.pitch_log.stop()
        finally:
            self.crazyflie.close_link()

    def connectionInitiated(self, linkURI):
        self.link_status = "Connection Initiated"
        self.link_status_pub.publish(self.link_status)

    def connectSetupFinished(self, linkURI):
        
        self.link_status = "Connect Setup Finished"
        self.link_status_pub.publish(self.link_status)
        
        self.setupStabilizerLog()

        """
        Configure the logger to log accelerometer values and start recording.
 
        The logging variables are added one after another to the logging
        configuration. Then the configuration is used to create a log packet
        which is cached on the Crazyflie. If the log packet is None, the
        program exits. Otherwise the logging packet receives a callback when
        it receives data, which prints the data from the logging packet's
        data dictionary as logging info.
        """
        # Set accelerometer logging config
        accel_log_conf = LogConfig("Accel", 10)
        accel_log_conf.addVariable(LogVariable("acc.x", "float"))
        accel_log_conf.addVariable(LogVariable("acc.y", "float"))
        accel_log_conf.addVariable(LogVariable("acc.z", "float"))
 
        # Now that the connection is established, start logging
        self.accel_log = self.crazyflie.log.create_log_packet(accel_log_conf)
 
        if self.accel_log is not None:
            self.accel_log.dataReceived.add_callback(self.log_accel_data)
            self.accel_log.start()
        else:
            print("acc.x/y/z not found in log TOC")


        # Set gyrometer logging config
        gyro_log_conf = LogConfig("Gyro", 10)
        gyro_log_conf.addVariable(LogVariable("gyro.x", "float"))
        gyro_log_conf.addVariable(LogVariable("gyro.y", "float"))
        gyro_log_conf.addVariable(LogVariable("gyro.z", "float"))
 
        # Now that the connection is established, start logging
        self.gyro_log = self.crazyflie.log.create_log_packet(gyro_log_conf)
 
        if self.gyro_log is not None:
            self.gyro_log.dataReceived.add_callback(self.log_gyro_data)
            self.gyro_log.start()
        else:
            print("gyro.x/y/z not found in log TOC")
        # Log barometer
        baro_log_conf = LogConfig("Baro", 200)
        baro_log_conf.addVariable(LogVariable("baro.aslLong", "float"))

        # Now that the connection is established, start logging
        self.baro_log = self.crazyflie.log.create_log_packet(baro_log_conf)
        if self.baro_log is not None:
            self.baro_log.dataReceived.add_callback(self.log_baro_data)
            self.baro_log.start()
        else:
            print("baro.aslLong not found in log TOC")


        # Set magnetometer logging config 
        #TODO: check functionality
        mag_log_conf = LogConfig("Magneto", 10)
        mag_log_conf.addVariable(LogVariable("mag.x", "float"))
        mag_log_conf.addVariable(LogVariable("mag.y", "float"))
        mag_log_conf.addVariable(LogVariable("mag.z", "float"))
 
        # Now that the connection is established, start logging
        self.mag_log = self.crazyflie.log.create_log_packet(mag_log_conf)
 
        if self.mag_log is not None:
            self.mag_log.dataReceived.add_callback(self.log_mag_data)
            self.mag_log.start()
        else:
            print("magneto.x/y/z not found in log TOC")

        motor_log_conf = LogConfig("Motor", 10)
        motor_log_conf.addVariable(LogVariable("motor.m1", "int32_t"))
        motor_log_conf.addVariable(LogVariable("motor.m2", "int32_t"))
        motor_log_conf.addVariable(LogVariable("motor.m3", "int32_t"))
        motor_log_conf.addVariable(LogVariable("motor.m4", "int32_t"))
 
        # Now that the connection is established, start logging
        self.motor_log = self.crazyflie.log.create_log_packet(motor_log_conf)
 
        if self.motor_log is not None:
            self.motor_log.dataReceived.add_callback(self.log_motor_data)
            self.motor_log.start()
        else:
            print("motor.m1/m2/m3/m4 not found in log TOC")
 
    def setupStabilizerLog(self):
        log_conf = LogConfig("Pitch", 10)
        log_conf.addVariable(LogVariable("stabilizer.pitch", "float"))
        log_conf.addVariable(LogVariable("stabilizer.roll", "float"))
        log_conf.addVariable(LogVariable("stabilizer.thrust", "int32_t"))
        log_conf.addVariable(LogVariable("stabilizer.yaw", "float"))
        self.pitch_log = self.crazyflie.log.create_log_packet(log_conf)
 
        if self.pitch_log is not None:
            self.pitch_log.dataReceived.add_callback(self.log_pitch_data)
            self.pitch_log.start()
            print("stabilizer.pitch/roll/thrust/yaw now logging")
        else:
            print("stabilizer.pitch/roll/thrust/yaw not found in log TOC")
        
    def connected(self, linkURI):
        self.packetsSinceConnection = 0
        self.link_status = "Connected"
        self.link_status_pub.publish(self.link_status)

    def disconnected(self, linkURI):
        self.link_status = "Disconnected"
        self.link_status_pub.publish(self.link_status)
     
    def connectionLost(self, linkURI, errmsg):
        self.link_status = "Connection Lost - " + errmsg
        self.link_status_pub.publish(self.link_status)
 
    def connectionFailed(self, linkURI, errmsg):
        self.link_status = "Connection Failed - " + errmsg
        self.link_status_pub.publish(self.link_status)
 
    def linkQuality(self, percentage):
        self.link_quality = percentage

    def receivedPacket(self, pk):
        self.packetsSinceConnection += 1

    def log_accel_data(self, data):
        rospy.loginfo("Accelerometer: x=%.2f, y=%.2f, z=%.2f" %
                        (data["acc.x"], data["acc.y"], data["acc.z"]))

    def log_gyro_data(self, data):
        rospy.loginfo("Gyrometer: x=%.2f, y=%.2f, z=%.2f" %
                        (data["gyro.x"], data["gyro.y"], data["gyro.z"]))

    def log_mag_data(self, data):
        rospy.loginfo("Magnetometer: x=%.2f, y=%.2f, z=%.2f" %
                        (data["mag.x"], data["mag.y"], data["mag.z"]))

    def log_baro_data(self, data):
        rospy.loginfo("Barometer: aslLong=%.2f" %
                        (data["baro.aslLong"]))

    def log_motor_data(self, data):
        self.motor_status = ("Motors: m1=%d, m2=%d, m3=%d, m4=%d" %
                        (data["m1"], data["m2"], data["m3"], data["m4"]))

    def log_pitch_data(self, data):
        self.pitch  = data["stabilizer.pitch"]
        self.roll   = data["stabilizer.roll"]
        self.thrust = data["stabilizer.thrust"]
        self.yaw    = data["stabilizer.yaw"]

    def set_thrust(self, data):
        rospy.loginfo(rospy.get_name() + ": Setting thrust to: %d" % data.data)
        self.cmd_thrust = data.data

    def run_node(self):
        self.link_quality_pub.publish(self.link_quality)
        self.packet_count_pub.publish(self.packetsSinceConnection)
        self.motor_status_pub.publish(self.motor_status)
        self.pitch_pub.publish(self.pitch)
        self.roll_pub.publish(self.roll)
        self.thrust_pub.publish(self.thrust)
        self.yaw_pub.publish(self.yaw)
        
        # Send commands to the Crazyflie
#        rospy.loginfo(rospy.get_name() + ": Sending setpoint: %f, %f, %f, %d" % (self.cmd_roll, self.cmd_pitch, self.cmd_yaw, self.cmd_thrust))
        self.crazyflie.commander.send_setpoint(self.cmd_roll, self.cmd_pitch, self.cmd_yaw, self.cmd_thrust)
         
def run():
    # Init the ROS node here, so we can split functionality
    # for this node across multiple classes        
    rospy.init_node('crazyflie')

    #TODO: organize this into several classes that monitor/control one specific thing
    node = CrazyflieNode()
    while not rospy.is_shutdown():
        node.run_node()
        rospy.sleep(0.1)
    node.shut_down()
        
        
if __name__ == '__main__':
    run()

