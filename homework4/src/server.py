#!/usr/bin/env python

import rospy
from homework4.srv import *

mode = rospy.get_param('mode', 0)

def handle_mode_status(req):
    return modeStatusResponse(mode)

def handle_mode_update(req):
    global mode
    mode = req.newmode
    return modeUpdateResponse(mode)

def mode_server():
    rospy.init_node('mode_server')
    mode_status = rospy.Service('mode_status', modeStatus, handle_mode_status)
    mode_update = rospy.Service('mode_update', modeUpdate, handle_mode_update)
    rospy.spin()

if __name__ == "__main__":
    mode_server()
