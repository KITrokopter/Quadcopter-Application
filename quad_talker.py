#!/usr/bin/env python
import rospy


def talker():
    pub = rospy.Publisher('chatter', quadcopter_application.)
    rospy.init_node('quad_talker')
    while not rospy.is_shutdown():
        quadcopter_application. quad_msg;
        pub.publish(quad_msg)
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
