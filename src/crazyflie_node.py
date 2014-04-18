#!/usr/bin/env python
#
# crazyflie_node.py
#
# Script for communication with a crazyflie and a ros server.
#
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

import os

roslib.load_manifest('quadcopter_application')

import cflib.crtp

from sets import Set
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import Log, LogVariable, LogConfig

from quadcopter_application.srv import *
from quadcopter_application.msg import *

from api_application.srv import Announce

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
LOGVARS_STABILIZER_INTERVALL = 20

LOGVARS_ACC = ['acc.x',
               'acc.y',
               'acc.z',
                    ]
LOGVARS_ACC_INTERVALL = 20

LOGVARS_BARO = ['baro.aslLong']
LOGVARS_BARO_INTERVALL = 20

LOGVARS_SYSTEM = ['pm.vbat']    
LOGVARS_SYSTEM_INTERVALL = 1000

class CrazyflieNode:

    ## Initialize the variables and the services and topics
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
        
        self.stabilizer_roll = 0.0
        self.stabilizer_pitch = 0.0
        self.stabilizer_yaw = 0.0
        self.stabilizer_thrust = 0
        
        self.motor_m1 = 0
        self.motor_m2 = 0
        self.motor_m3 = 0
        self.motor_m4 = 0
        
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
        
        self.estimated_vel_x = 0.0
        self.estimated_vel_y = 0.0
        
        self.baro = 0.0
        #TODO: quadcopter stabilizer
        #self.target_baro = 83.9
        
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
    
    ## Announce the module at the api to get an id
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
    
    ## Initialize the service to search for available channels
    def init_search_links_service(self):
        rospy.Service('search_links_' + str(self.id), search_links, self.handle_search_links)
    
    ## Handle a search link request
    #  @param req the request of the ros service
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
    
    ## Handle a open link request which should lead to a connection with a crazylie
    #  @param req the request of the ros service
    def handle_open_link(self, req):
        self.link_channel = req.channel
        self.crazyflie.open_link("radio://" + str(self.dongle_id) + "/" + str(self.link_channel) + "/250K")
        rospy.loginfo("Opened link with uri " + "radio://" + str(self.dongle_id) + "/" + str(self.link_channel) + "/250K")
        
        #init the ROS topic for controlling the quadcopter
        rospy.Subscriber('quadcopter_movement_' + str(self.id), quadcopter_movement, self.set_movement)
        return open_linkResponse(0)

    ## Initialize the open link service
    def init_open_link_service(self):
        #service for opening a link to a quadcopter
        s = rospy.Service('open_link_' + str(self.id), open_link, self.handle_open_link)
        rospy.loginfo("Ready to open a link to a quadcopter.")
    
    ## Handle a close link request
    #  @param req the ros service request
    def handle_close_link(self, req):
        shut_down()

    ## Initialize the close link service
    def init_close_link_service(self):
        #service for closing the link to the quadcopter
        s = rospy.Service('close_link_' + str(self.id), close_link, self.handle_close_link)
        rospy.loginfo("Ready to close a link to a quadcopter.")

    ## Handle a close link request
    # @param req the ros service request
    def handle_blink(self, req):
        #set the trust to make the rotors spin but not to start the quadcopter
        self.cmd_thrust = 20000
        self.run_node()
        rospy.sleep(2.0)
        self.cmd_thrust = 0
        self.run_node()
        return blinkResponse(0)

    ## Initialize the blink service
    def init_blink_service(self):
        #service for blinking to see which quadcopter is managed
        s = rospy.Service('blink_' + str(self.id), blink, self.handle_blink)
        rospy.loginfo("Ready to blink.")

    ## Shut down the connection to the crazyflie
    def shut_down(self):
        rospy.loginfo("Shutting down")
        self.crazyflie.close_link()
    
    ## When there is an error with logging the crazyflie sensor vars
    # @param data the data provided for the error (currently not used)
    def onLogError(self, data):
        #TODO use the data var
        rospy.logerror("Log error")
    
    ## Handler when receiving a new motor var package
    #  @param timestamp the timestamp of the log package
    #  @param data the log values
    #  @param logconf the log configuration
    def on_log_data_motor(self, timestamp, data, logconf):
        self.motor_m1 = data['motor.m1']
        self.motor_m1 = data['motor.m2']
        self.motor_m1 = data['motor.m3']
        self.motor_m1 = data['motor.m4']
    
    ## Handler when receiving a new stabilizer var package
    #  @param timestamp the timestamp of the log package
    #  @param data the log values
    #  @param logconf the log configuration
    def on_log_data_stabilizer(self, timestamp, data, logconf):
        self.stabilizer_roll = data['stabilizer.roll']
        self.stabilizer_pitch = data['stabilizer.pitch']
        self.stabilizer_yaw = data['stabilizer.yaw']
        self.stabilizer_thrust = data['stabilizer.thrust']
    
    ## Handler when receiving a new accelerometer var package
    #  @param timestamp the timestamp of the log package
    #  @param data the log values
    #  @param logconf the log configuration
    def on_log_data_acc(self, timestamp, data, logconf):
        self.acc_x = data['acc.x']
        self.acc_y = data['acc.y']
        self.acc_z = data['acc.z']
    
    ## Handler when receiving a new barometer var package
    #  @param timestamp the timestamp of the log package
    #  @param data the log values
    #  @param logconf the log configuration
    def on_log_data_baro(self, timestamp, data, logconf):
        self.baro = data['baro.aslLong']    
    
    ## Handler when receiving a new system var package
    #  @param timestamp the timestamp of the log package
    #  @param data the log values
    #  @param logconf the log configuration
    def on_log_data_system(self, timestamp, data, logconf):
        self.battery_status = data['pm.vbat']

    ## Handler when set up a new connection. This will start logging the sensor vars.
    #  @param linkURI the uri of the connection to the crazyflie
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
        
        lgAcc = LogConfig("logAcc", LOGVARS_ACC_INTERVALL)
        for f in LOGVARS_ACC:
            lgAcc.add_variable(f)
        self.crazyflie.log.add_config(lgAcc)
        
        lgBa = LogConfig("logBa", LOGVARS_BARO_INTERVALL)
        for f in LOGVARS_BARO:
            lgBa.add_variable(f)
        self.crazyflie.log.add_config(lgBa)
        
        lgSy = LogConfig("logSy", LOGVARS_SYSTEM_INTERVALL)
        for f in LOGVARS_SYSTEM:
            lgSy.add_variable(f)
        self.crazyflie.log.add_config(lgSy)
        
        if (lgM.valid and lgSt.valid and lgAcc.valid and lgBa.valid and lgSy.valid):
            lgM.data_received_cb.add_callback(self.on_log_data_motor)
            lgSt.data_received_cb.add_callback(self.on_log_data_stabilizer)
            lgAcc.data_received_cb.add_callback(self.on_log_data_acc)
            lgBa.data_received_cb.add_callback(self.on_log_data_baro)
            lgSy.data_received_cb.add_callback(self.on_log_data_system)

            lgM.error_cb.add_callback(self.onLogError)
            lgSt.error_cb.add_callback(self.onLogError)
            lgAcc.error_cb.add_callback(self.onLogError)
            lgBa.error_cb.add_callback(self.onLogError)
            lgSy.error_cb.add_callback(self.onLogError)

            lgM.start()
            lgSt.start()
            lgAcc.start()
            lgBa.start()
            lgSy.start()
            print("Crazyflie sensor log successfully started")
            rospy.loginfo("Crazyflie sensor log successfully started")
        else:
            rospy.logerror("Error while starting crazyflie sensr logs")
            logger.warning("Could not setup logconfiguration after connection!")
            print("Logging failed")

    ## Handler when disconnected from crazyflie
    #  @param linkURI the uri of the connection to the crazyflie
    def disconnected(self, linkURI):
        self.link_status = "Disconnected"
    
    ## Handler when lost connection to crazyflie
    #  @param linkURI the uri of the connection to the crazyflie
    #  @param errmsg the error msg provided by the cflib
    def connectionLost(self, linkURI, errmsg):
        self.link_status = "Connection Lost - " + errmsg
        rospy.logerror("Lost connection to quadcopter")

    ## Handler when the connection to the crazyflie failed
    #  @param linkURI the uri of the connection to the crazyflie
    #  @param errmsg the error msg provided by the cflib
    def connectionFailed(self, linkURI, errmsg):
        self.link_status = "Connection Failed - " + errmsg
        rospy.logerror("Connection to quadcopter failed")

    ## Handler when getting a new linkQuality value
    #  @param percentage the link quality
    def linkQuality(self, percentage):
        self.link_quality = percentage

    ## Handler when received a new packet from the crazyflie
    #  @param pk the package
    def receivedPacket(self, pk):
        self.packetsSinceConnection += 1

    ## Handler for getting a command to controll the crazyflie's movement
    #  @param data the data of the ros message
    def set_movement(self, data):
        self.cmd_thrust = data.thrust
        self.cmd_roll = data.roll
        self.cmd_pitch = data.pitch
        self.cmd_yaw = data.yaw
    
    ## Testcode for selfstabilizing the crazyflie
    def stabilize(self):
        self.estimated_vel_x = 0.85 * self.estimated_vel_x + self.acc_x
        self.estimated_vel_y = 0.85 * self.estimated_vel_y + self.acc_y
        if (self.baro != 0.0):
            self.cmd_thrust += (self.target_baro - self.baro) * 50 * (48000/(self.cmd_thrust + 1))
            self.cmd_pitch = -self.estimated_vel_y * 12 + self.acc_y * 2
            self.cmd_roll = -self.estimated_vel_x * 12 + self.acc_x * 2

    ## Send the last logged vars over the ros topic and and the commands to crazyflie.
    #  Needs to be called repetetive.
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
        
        msg.baro = self.baro
        
        msg.acc_x = self.acc_x
        msg.acc_y = self.acc_y
        msg.acc_z = self.acc_z

        self.publisher.publish(msg)
        # Send commands to the Crazyflie
        # DEBUG
        #rospy.loginfo(rospy.get_name() + ": Sending setpoint: %f, %f, %f, %d" %
        #       (self.cmd_roll, self.cmd_pitch, self.cmd_yaw, self.cmd_thrust))
        #self.stabilize()
        
        os.system('clear')
        print("acc_x: " + str(self.acc_x))
        print("acc_y: " + str(self.acc_y))
        print("acc_z: " + str(self.acc_z))
        print("baro: " + str(self.baro))
        
        print("\n===============commands======================\n");
        print("thrust: " + str(self.cmd_thrust))
        print("roll: " + str(self.cmd_roll))
        print("pitch: " + str(self.cmd_pitch))
        
        print("\n===============estimated velocity======================\n");
        print("x: " + str(self.estimated_vel_x))
        print("y: " + str(self.estimated_vel_y))
        self.crazyflie.commander.send_setpoint(self.cmd_roll, self.cmd_pitch, self.cmd_yaw, self.cmd_thrust)
        
def run():
    rospy.init_node('crazyflie', anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo("ros node successfully initialized")

    node = CrazyflieNode()
    rospy.loginfo("crazyflie node successfully initialized")
    while not rospy.is_shutdown():
        node.run_node()
        rospy.sleep(0.03)
    node.shut_down()
        
        
if __name__ == '__main__':
    run()
