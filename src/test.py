#!/usr/bin/env python2.7
import time
import threading
import logging
import cflib.crtp
from cflib import crazyflie
from cfclient.utils.logconfigreader import LogVariable, LogConfig

print("hallow world!")

logger = logging.getLogger()

INITIAL_ROLL = 0.0
INITIAL_PITCH = 0.0
INITIAL_YAWRATE = 0
INITIAL_THRUST = 10001

class CrazyDemo(object):
    def __init__(self):
        self.point = {'roll': INITIAL_ROLL,
                      'pitch': INITIAL_PITCH,
                      'yawrate': INITIAL_YAWRATE,
                      'thrust': INITIAL_THRUST}
        cflib.crtp.init_drivers()
        self.cf = crazyflie.Crazyflie()
        self.cf.connectSetupFinished.add_callback(self.connected)
        self.cf.disconnected.add_callback(self.disconnected)
        self.cf.open_link("radio://0/11/250K")
        # self.cf.open_link("debug://0/0")
        self.stopevent = threading.Event()
        self.sp = SendPoints(self.cf, self.stopevent, self.point)

    def start(self):
        import sys
        range = 350
        factor = 1000
        current = INITIAL_THRUST
        self.sp.start()
        #for i in xrange(range):
            # self.point['thrust'] = 0
            #self.point['thrust'] = INITIAL_THRUST + (factor * i)
            #time.sleep(0.01)
        self.point['thrust'] = self.point['thrust'] + 20000
        while 1:
            ch = sys.stdin.read(1)    
            if ch == "u":
                self.point['thrust'] = self.point['thrust'] + factor
            elif ch == "e":
                self.point['thrust'] = self.point['thrust'] - (3*factor)
                time.sleep(0.5)
                self.point['thrust'] = self.point['thrust'] - (3*factor)
                time.sleep(0.5)
                self.point['thrust'] = self.point['thrust'] - (3*factor)
                time.sleep(0.5)
                self.stop();
            elif ch == "d":
                self.point['thrust'] = self.point['thrust'] - factor
            print self.point
            time.sleep(0.05)

    def connected(self, link_uri):
        print 'crazyflie setup complete %s' % link_uri
        stabilizerconf = LogConfig("Stabilizer", 10)
        stabilizerconf.addVariable(LogVariable("stabilizer.roll"))
        stabilizerconf.addVariable(LogVariable("stabilizer.pitch"))
        stabilizerconf.addVariable(LogVariable("stabilizer.yaw"))
        stabilizerconf.addVariable(LogVariable("stabilizer.thrust"))
        self.stabilizerlog = self.cf.log.create_log_packet(stabilizerconf)

        if self.stabilizerlog is not None:
            self.stabilizerlog.dataReceived.add_callback(self.stabilizer_data)
            self.stabilizerlog.start()
        else:
            print "stabilzer not found in log TOC"

    def disconnected(self, link_uri):
        # self.stabilizerlog.dataReceived.remove_callback()
        self.stabilizerlog.stop()

    def stabilizer_data(self, data):
        #print 'roll', data['stabilizer.roll']
        #print 'pitch', data['stabilizer.pitch']
	print "hello"	

    def stop(self):
        print 'closing link'
        self.stopevent.set()
        self.cf.close_link()

    def __del__(self):
        self.stop()


class SendPoints(threading.Thread):
    def __init__(self, cf, stopevent, point):
        threading.Thread.__init__(self, name='sendpoints')
        self.stopevent = stopevent
        self.cf = cf
        self.point = point

    def run(self):
        logger.debug("sendpoints thread started")
        while not self.stopevent.is_set():
            #print self.point
            self.cf.commander.send_setpoint(self.point['roll'],
                                            self.point['pitch'],
                                            self.point['yawrate'],
                                            self.point['thrust'])
            self.stopevent.wait(0.02)
        print 'stopping sendpoint thread'


c = CrazyDemo()
c.start()
time.sleep(1)
c.stop()

