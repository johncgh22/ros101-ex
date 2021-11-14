#!/usr/bin/env python3

import rospy
from receiver_pkg.msg import prueba_status2

def talker2():
    pub2 = rospy.Publisher('status2', prueba_status2, queue_size=10)
    rospy.init_node('second_talker', anonymous=True)
    r2 = rospy.Rate(20)  # 20 Hz
    msg2 = prueba_status2()
    counter = 0
    counter += 1 
    msg2.data = 'Se esta publicando el numero %d'%counter
    
    while not rospy.is_shutdown():
        rospy.loginfo(msg2)
        pub2.publish(msg2)
        r2.sleep()    

if __name__=='__main__':
    talker2()