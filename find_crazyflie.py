#!/usr/bin/env python2.7
import time
import threading
import logging
import cflib.crtp
from cflib import crazyflie
from cfclient.utils.logconfigreader import LogVariable, LogConfig

print("start")
cflib.crtp.init_drivers()
available = cflib.crtp.scan_interfaces()
for i in available:
	print "InterfacewithURI [%s] found, name [%s]" % (i[0],i[1])

print("end")
