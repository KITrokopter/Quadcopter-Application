Quadcopter-Application
==============

quadcopter module
--------------
Needed for python program as library:
hg clone https://bitbucket.org/bitcraze/crazyflie-pc-client

http://www.debian.org/doc/packaging-manuals/python-policy/ch-python.html
"Public Python 3 modules must be installed in /usr/lib/python3/dist-packages..."


Startprocedure for Quadcopter-applications

Starting roscore
Starting a master-node "master". The master-application will make the decision which application connects with which quadcopter
Starting other nodes "nodes". Each will start with a unique ID (alternative: generate a unique ID, but the master shouldn't worry about the uniqueness of each application-ID)
The nodes will ask the master to be put in the list of running node-applications (a Service).
The master will find the URIs (somehow) and call each node to open a link to one quadcopter (another Service) 
The nodes (and probably the master too) will be Subscribers to the controll-messages (the topic, respectively).


Shuting down
TODO

Error
TODO (scenarios in Pflichtenheft?)

