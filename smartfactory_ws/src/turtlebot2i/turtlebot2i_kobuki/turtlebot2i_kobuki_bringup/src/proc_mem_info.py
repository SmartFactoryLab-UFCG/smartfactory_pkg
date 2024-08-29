#!/usr/bin/env python

import rospy
import psutil
from std_msgs.msg import Float32

def publisher():
    proc_pub = rospy.Publisher('/mobile_base/sensors/cpu', Float32, queue_size=1)
    mem_pub = rospy.Publisher('/mobile_base/sensors/mem', Float32, queue_size=1)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        proc_pub.publish(psutil.cpu_percent())
        mem_pub.publish(psutil.virtual_memory().percent)

        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
