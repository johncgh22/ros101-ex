#!/usr/bin/env python

import rospy
from receiver_pkg.msg import Status
from receiver_pkg.msg import prueba_status2

def callback(data):
    rospy.loginfo('El estado del Robot es {} - {}'.format(data.code, data.name))
                  #'El estado del Robot es %d %s', (data.code, data.name)

def callback2(data):
    rospy.loginfo('Estoy leyendo el valor %d'.format(data.counter))


def listener():
    rospy.init_node('my_listener', anonymous=True)
    # rospy.init_node('my_second_listener', anonymous=True)

    rospy.Subscriber('status_info', Status, callback)
    #rospy.Subscriber('status2', prueba_status2, callback2)

    rospy.spin()

if __name__=='__main__':
    listener()