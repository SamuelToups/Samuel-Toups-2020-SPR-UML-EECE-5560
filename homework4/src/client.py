#!/usr/bin/env python

import rospy
from homework4.srv import *


if __name__ == "__main__":
    rospy.wait_for_service('mode_status')
    rospy.wait_for_service('mode_update')
    n = 0
    try:
        rospy.init_node('mode_client', anonymous=True)
        rate = rospy.Rate(2)
        mode_status = rospy.ServiceProxy('mode_status', modeStatus)
        mode_update = rospy.ServiceProxy('mode_update', modeUpdate)
        while not rospy.is_shutdown():
            try:
                mode_update(n)
                rospy.loginfo("mode_client: mode changed to %s", mode_status().mode)
                n = n + 1
            except rospy.ServiceException, e:
                rospy.loginfo("mode_client: Service call failed: %s"%e)
                rate.sleep
    except rospy.ROSInterruptException:
        pass
