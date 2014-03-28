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

from sets import Set
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import Log, LogVariable, LogConfig

from quadcopter_application.srv import *
from quadcopter_application.msg import *

from api_application.srv import Announce

#TODO why do we need this import?
from control_application.msg import *

logging.basicConfig(level=logging.DEBUG)

#variables which will be received from the quadcopters
#split into multiple packages since they would be to big for one single package 
LOGVARS_MOTOR = ['motor.m1',
                'motor.m2',
                'motor.m3',
                'motor.m4'
                ]
#time to wait between to packages of this type in ms
LOGVARS_MOTOR_INTERVALL = 100

LOGVARS_STABILIZER = ['stabilizer.roll',
                    'stabilizer.pitch',
                    'stabilizer.yaw',
                    'stabilizer.thrust'
                    ]
LOGVARS_STABILIZER_INTERVALL = 50

LOGVARS_SYSTEM = ['pm.vbat']    
LOGVARS_SYSTEM_INTERVALL = 500

class CrazyflieNode:

    def __init__(self):
        rospy.loginfo("Init started")

        #Connect to Crazyflie, initialize drivers and set up callback.

        #the module id which this apllication gets from the api
        self.id = 0
        self.retrieve_id();
        
        self.dongle_id = 0
        
        if rospy.has_param('dongle_id'):
            self.dongle_id = rospy.get_param('dongle_id')
        
        self.link_channel = 10
        
        self.link_status = "Unknown"
        self.link_quality = 0.0
        self.battery_status = 0.0
        self.packetsSinceConnection = 0
        
        #currently there is no direct access to the sensor values
        self.stabilizer_roll = 0.0
        self.stabilizer_pitch = 0.0
        self.stabilizer_yaw = 0.0
        self.stabilizer_thrust = 0
        
        self.motor_m1 = 0
        self.motor_m2 = 0
        self.motor_m3 = 0
        self.motor_m4 = 0
        
        #commands are stored to send them with the next package
        self.cmd_roll = 0.0
        self.cmd_pitch = 0.0
        self.cmd_yaw = 0.0
        self.cmd_thrust = 0
        
        # Init the callbacks for the crazyflie lib
        self.crazyflie = Crazyflie()
        cflib.crtp.init_drivers()

        #init the status publisher topic for ROS
        self.publisher  = rospy.Publisher('quadcopter_status_' + str(self.id), quadcopter_status, latch=True)

        # TODO Which callbacks are still needed?
        # Connection callbacks
        self.crazyflie.connected.add_callback(self.connected)
        self.crazyflie.disconnected.add_callback(self.disconnected)
        self.crazyflie.connection_lost.add_callback(self.connectionLost)
        self.crazyflie.connection_failed.add_callback(self.connectionFailed)

        # Link quality callbacks
        self.crazyflie.link_quality_updated.add_callback(self.linkQuality)
        self.crazyflie.packet_received.add_callback(self.receivedPacket)
        
        rospy.loginfo("All crazyflie callbacks successfully initialized")
        
        #initialize the services
        self.init_search_links_service()
        self.init_open_link_service()
        self.init_close_link_service()
        self.init_blink_service()
        
        rospy.loginfo("All services successfully initialized")
    
    #get the module id from the api
    def retrieve_id(self):
        rospy.loginfo("Waiting for announce service")
        rospy.wait_for_service('announce')
        try:
            retrieve_id_service = rospy.ServiceProxy('announce', Announce)
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            response = retrieve_id_service(h, 1, 0)
            if response.id == -1:
                rospy.logerr("Error during announcement")
            else:
                self.id = response.id
                rospy.loginfo("Got id %d", self.id)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e      
        
    def init_search_links_service(self):
        rospy.Service('search_links_' + str(self.id), search_links, self.handle_search_links)
        
    def handle_search_links(self, req):
        available = cflib.crtp.scan_interfaces()
        channels = list()
        for uri in available:
            #we get an uri like "radio://0/10/250K" and have to split it to get the channel 
            splitted = uri[0].split('/')
            rospy.loginfo("Found crazyflie with uri " + uri[0])
            rospy.loginfo("try to add " + splitted[3])
            channels.append(int(splitted[3]))
        return search_linksResponse(channels)
    
    def handle_open_link(self, req):
        self.link_channel = req.channel
        self.crazyflie.open_link("radio://" + str(self.dongle_id) + "/" + str(self.link_channel) + "/250K")
        rospy.loginfo("Opened link with uri " + "radio://" + str(self.dongle_id) + "/" + str(self.link_channel) + "/250K")
        
        #init the ROS topic for controlling the quadcopter
        rospy.Subscriber('quadcopter_movement_' + str(self.id), quadcopter_movement, self.set_movement)
        return open_linkResponse(0)

    def init_open_link_service(self):
        #service for opening a link to a quadcopter
        s = rospy.Service('open_link_' + str(self.id), open_link, self.handle_open_link)
        rospy.loginfo("Ready to open a link to a quadcopter.")
        
    def handle_close_link(self, req):
        shut_down()

    def init_close_link_service(self):
        #service for closing the link to the quadcopter
        s = rospy.Service('close_link_' + str(self.id), close_link, self.handle_close_link)
        rospy.loginfo("Ready to close a link to a quadcopter.")

    def handle_blink(self, req):
        #set the trust to make the rotors spin but not to start the quadcopter
        self.cmd_thrust = 20000
        self.run_node()
        rospy.sleep(2.0)
        self.cmd_thrust = 0
        self.run_node()
        return blinkResponse(0)

    def init_blink_service(self):
        #service for blinking to see which quadcopter is managed
        s = rospy.Service('blink_' + str(self.id), blink, self.handle_blink)
        rospy.loginfo("Ready to blink.")

    def shut_down(self):
        rospy.loginfo("Shutting down")
        self.crazyflie.close_link()

    def onLogError(self, data):
        rospy.logerror("Log error")
    
    def on_log_data_motor(self, timestamp, data, logconf):
        self.motor_m1 = data['motor.m1']
        self.motor_m1 = data['motor.m2']
        self.motor_m1 = data['motor.m3']
        self.motor_m1 = data['motor.m4']
        
    def on_log_data_stabilizer(self, timestamp, data, logconf):
        self.stabilizer_roll = data['stabilizer.roll']
        self.stabilizer_pitch = data['stabilizer.pitch']
        self.stabilizer_yaw = data['stabilizer.yaw']
        self.stabilizer_thrust = data['stabilizer.thrust']
        
    def on_log_data_system(self, timestamp, data, logconf):
        self.battery_status = data['pm.vbat']

    def connected(self, linkURI):
        self.packetsSinceConnection = 0
        self.link_status = "Connected"
        print("Connected") #DEBUG
        
        """
        Configure the logger and start recording.

        The logging variables are added one after another to the logging
        configuration. Then the configuration is used to create a log packet
        which is cached on the Crazyflie. If the log packet is None, the
        program exits. Otherwise the logging packet receives a callback when
        it receives data, which prints the data from the logging packet's
        data dictionary as logging info.
        """
        lgM = LogConfig("logM", LOGVARS_MOTOR_INTERVALL)
        for f in LOGVARS_MOTOR:
            lgM.add_variable(f)
        self.crazyflie.log.add_config(lgM)
        
        lgSt = LogConfig("logSt", LOGVARS_STABILIZER_INTERVALL)
        for f in LOGVARS_STABILIZER:
            lgSt.add_variable(f)
        self.crazyflie.log.add_config(lgSt)
        
        lgSy = LogConfig("logSy", LOGVARS_SYSTEM_INTERVALL)
        for f in LOGVARS_SYSTEM:
            lgSy.add_variable(f)
        self.crazyflie.log.add_config(lgSy)
        
        if (lgM.valid and lgSt.valid and lgSy.valid):
            lgM.data_received_cb.add_callback(self.on_log_data_motor)
            lgSt.data_received_cb.add_callback(self.on_log_data_stabilizer)
            lgSy.data_received_cb.add_callback(self.on_log_data_system)

            lgM.error_cb.add_callback(self.onLogError)
            lgSt.error_cb.add_callback(self.onLogError)
            lgSy.error_cb.add_callback(self.onLogError)

            lgM.start()
            lgSt.start()
            lgSy.start()
            rospy.loginfo("Crazyflie sensor log successfully started")
        else:
            rospy.logerror("Error while starting crazyflie sensr logs")
            logger.warning("Could not setup logconfiguration after connection!")

    def disconnected(self, linkURI):
        self.link_status = "Disconnected"
    
    def connectionLost(self, linkURI, errmsg):
        self.link_status = "Connection Lost - " + errmsg
        rospy.logerror("Lost connection to quadcopter")

    def connectionFailed(self, linkURI, errmsg):
        self.link_status = "Connection Failed - " + errmsg
        rospy.logerror("Connection to quadcopter failed")

    def linkQuality(self, percentage):
        self.link_quality = percentage

    def receivedPacket(self, pk):
        self.packetsSinceConnection += 1

    def set_movement(self, data):
        self.cmd_thrust = data.thrust
        self.cmd_roll = data.roll
        self.cmd_pitch = data.roll
        self.cmd_yaw = data.yaw

    def run_node(self):
        msg = quadcopter_status()
        msg.header.stamp = rospy.Time.now()
        
        msg.battery_status = self.battery_status
        msg.link_quality = self.link_quality

        msg.motor_m1 = self.motor_m1
        msg.motor_m2 = self.motor_m2
        msg.motor_m3 = self.motor_m3
        msg.motor_m4 = self.motor_m4

        msg.stabilizer_roll = self.stabilizer_roll
        msg.stabilizer_pitch = self.stabilizer_pitch
        msg.stabilizer_yaw = self.stabilizer_yaw
        msg.stabilizer_thrust = self.stabilizer_thrust

        self.publisher.publish(msg)
        # Send commands to the Crazyflie
        # DEBUG
        #rospy.loginfo(rospy.get_name() + ": Sending setpoint: %f, %f, %f, %d" %
        #       (self.cmd_roll, self.cmd_pitch, self.cmd_yaw, self.cmd_thrust))
        self.crazyflie.commander.send_setpoint(self.cmd_roll, self.cmd_pitch, self.cmd_yaw, self.cmd_thrust)
        
def run():
    rospy.init_node('crazyflie', log_level=rospy.DEBUG)
    rospy.loginfo("ros node successfully initialized")

    node = CrazyflieNode()
    rospy.loginfo("crazyflie node successfully initialized")
    while not rospy.is_shutdown():
        node.run_node()
        rospy.sleep(0.05)
    node.shut_down()
        
        
if __name__ == '__main__':
    run()
