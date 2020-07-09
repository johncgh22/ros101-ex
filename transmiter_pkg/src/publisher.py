#!/usr/bin/env python

import rospy
from receiver_pkg.msg import Status
from receiver_pkg.msg import prueba_status2

def talker():
    pub = rospy.Publisher('status_info', Status ,queue_size=10)
    rospy.init_node('my_talker', anonymous=True)
    r = rospy.Rate(10)  #10 Hz
    msg = Status()
    msg.name = 'Ok'
    msg.code = 100

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

def talker2():
    pub2 = rospy.Publisher('status2', prueba_status2, queue_size=10)
    rospy.init_node('second_talker', anonymous=True)
    r2 = rospy.Rate(20)  # 20 Hz
    msg2 = prueba_status2()
    counter = 0
    msg2.name = 'Se esta publicando el numero %d'%counter
    counter += 1 

        while not rospy.is_shutdown():
        rospy.loginfo(msg2)
        pub.publish(msg2)
        r.sleep()    

if __name__=='__main__':
    talker()
    talker2()