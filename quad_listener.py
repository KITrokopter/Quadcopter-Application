#!/usr/bin/env python
#!/usr/bin/env python
import rospy


def movement_callback(data):
    rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)
	self.cmd_thrust = data.thrust
	self.cmd_pitch = data.pitch
	self.cmd_yaw = data.yaw
	self.cmd_roll = data.roll


def listener():
    rospy.init_node('quad_listener', anonymous=True)
    rospy.Subscriber("Movement", quadcopter_application.Movement, movement_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()

