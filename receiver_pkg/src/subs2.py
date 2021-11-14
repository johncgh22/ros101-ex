#!/usr/bin/env python3

import rospy
from receiver_pkg.msg import prueba_status2

def callback2(data):
    rospy.loginfo('Estoy leyendo el valor %d'.format(data.counter))

def listener2():
    rospy.init_node('my_second_listener', anonymous=True)
    rospy.Subscriber('status2', prueba_status2, callback2)

    rospy.spin()

if __name__=='__main__':
    listener2()