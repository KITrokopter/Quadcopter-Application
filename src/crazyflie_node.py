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

import rospy
import logging
 
import cflib.crtp
from cfclient.utils.logconfigreader import LogConfig
from cfclient.utils.logconfigreader import LogVariable
from cflib.crazyflie import Crazyflie
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Float32
from std_msgs.msg import String
 
logging.basicConfig(level=logging.DEBUG)

class CrazyflieNode:
    """
    Class is required so that methods can access the object fields.
    """
    def __init__(self):
        """
        Connect to Crazyflie, initialize drivers and set up callback.
 
        The callback takes care of logging the accelerometer, altimeter, barometer, gyrometer and magnetometer values.
        """
        
        self.link_status = "Unknown"
        self.link_quality = 0.0
        self.battery_status = 0.0
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
        
        # Init the callbacks for the crazyflie lib
        self.crazyflie = Crazyflie()
        cflib.crtp.init_drivers()
 	
        # Connection callbacks
        self.crazyflie.connectionInitiated.add_callback(self.connectionInitiated)
        self.crazyflie.connectSetupFinished.add_callback(self.connectSetupFinished)
        self.crazyflie.connected.add_callback(self.connected)
        self.crazyflie.disconnected.add_callback(self.disconnected)
        self.crazyflie.connectionLost.add_callback(self.connectionLost)
        self.crazyflie.connectionFailed.add_callback(self.connectionFailed)

        # Link quality callbacks
        self.crazyflie.linkQuality.add_callback(self.linkQuality)
        self.crazyflie.batteryStatus.add_callback(self.batteryStatus)
        self.crazyflie.receivedPacket.add_callback(self.receivedPacket)
        
        #TODO: should be configurable, and support multiple devices
        self.crazyflie.open_link("radio://0/10/250K")
 
        #TODO: Test Start 
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
            self.pitch_log.stop()
        finally:
            self.crazyflie.close_link()

    def connectionInitiated(self, linkURI):
        self.link_status = "Connection Initiated"

    def connectSetupFinished(self, linkURI):
        
        self.link_status = "Connect Setup Finished"
        
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

        # Log altimeter
        #TODO: check functionality
        alti_log_conf = LogConfig("Alti", 200)
        alti_log_conf.addVariable(LogVariable("alti.aslLong", "float"))

        # Now that the connection is established, start logging
        self.alti_log = self.crazyflie.log.create_log_packet(alti_log_conf)
        if self.alti_log is not None:
            self.alti_log.dataReceived.add_callback(self.log_alti_data)
            self.alti_log.start()
        else:
            print("alti.aslLong not found in log TOC")

        #Log motor
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
		#TODO Why pitch_log?
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

    def disconnected(self, linkURI):
        self.link_status = "Disconnected"
     
    def connectionLost(self, linkURI, errmsg):
        self.link_status = "Connection Lost - " + errmsg
 
    def connectionFailed(self, linkURI, errmsg):
        self.link_status = "Connection Failed - " + errmsg
 
    def linkQuality(self, percentage):
        self.link_quality = percentage

	#TODO Check
    def batteryStatus(self, percentage):
        self.battery_status = percentage

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

    def log_alti_data(self, data):
        rospy.loginfo("Altimeter: aslLong=%.2f" %
                        (data["alti.aslLong"]))

    def log_motor_data(self, data):
        self.motor_status = ("Motors: m1=%d, m2=%d, m3=%d, m4=%d" %
                        (data["m1"], data["m2"], data["m3"], data["m4"]))

    def log_pitch_data(self, data):
        self.pitch  = data["stabilizer.pitch"]
        self.roll   = data["stabilizer.roll"]
        self.thrust = data["stabilizer.thrust"]
        self.yaw    = data["stabilizer.yaw"]

   
	def movement_callback(data):
   		rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)
		self.cmd_thrust = data.thrust
		self.cmd_pitch = data.pitch
		self.cmd_yaw = data.yaw
		self.cmd_roll = data.roll

          
def talker():
	pub_QuadStatus = rospy.Publisher('QuadStatus', quadcopter_application.QuadStatus)
	rospy.init_node('quad_talker')
	node = CrazyflieNode()
    while not rospy.is_shutdown():
		#TODO Globale Variablen? ID?
     	quad_msg.linkQuality = self.link_quality
		quad_msg.batteryStatus = self.battery_status
		quad_msg.accX = self.log_accel_data["acc.x"]
		quad_msg.accY = self.log_accel_data["acc.y"]
		quad_msg.accZ = self.log_accel_data["acc.z"]
		quad_msg.gyroX = self.log_gyro_data["gyro.x"]
		quad_msg.gyroY = self.log_gyro_data["gyro.y"]
		quad_msg.gyroZ = self.log_gyro_data["gyro.y"]
		quad_msg.magX = self.log_mag_data["mag.x"]
		quad_msg.magY = self.log_mag_data["mag.y"]
		quad_msg.magZ = self.log_mag_data["mag.z"]
		quad_msg.barometer = data["baro.aslLong"]
		quad_msg.altimeter = data["alti.aslLong"]
		pub_QuadStatus.publish(quad_msg)
        rospy.sleep(1.0)



def listener():
  	rospy.init_node('quad_listener', anonymous=True)
	node = CrazyflieNode()
    rospy.Subscriber("Movement", quadcopter_application.Movement, movement_callback)
   	rospy.spin()
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
	listener()
	# Send commands to the Crazyflie
	#rospy.loginfo(rospy.get_name() + ": Sending setpoint: %f, %f, %f, %d" % (self.cmd_roll, self.cmd_pitch, self.cmd_yaw, self.cmd_thrust))
        self.crazyflie.commander.send_setpoint(self.cmd_roll, self.cmd_pitch, self.cmd_yaw, self.cmd_thrust)
	
