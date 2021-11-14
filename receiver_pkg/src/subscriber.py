#!/usr/bin/env python3

import rospy
from receiver_pkg.msg import Status

def callback(data):
    rospy.loginfo('El estado del Robot es {} - {}'.format(data.code, data.name))
                  #'El estado del Robot es %d %s', (data.code, data.name)

def listener():
    rospy.init_node('my_listener', anonymous=True)
    rospy.Subscriber('status_info', Status, callback)

    rospy.spin()

if __name__=='__main__':
    listener()