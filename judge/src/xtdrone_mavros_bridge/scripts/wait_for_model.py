#!/usr/bin/env python3
import os
import sys
import time

import rospy
from gazebo_msgs.srv import GetModelProperties

if len(sys.argv) < 3:
    raise SystemExit("usage: wait_for_model.py <model_name> <command> [args...]")

model_name = sys.argv[1]
command = sys.argv[2:]
rospy.init_node("wait_for_model_" + model_name, anonymous=True)
rospy.wait_for_service("/gazebo/get_model_properties")
get_model_properties = rospy.ServiceProxy("/gazebo/get_model_properties", GetModelProperties)

while not rospy.is_shutdown():
    try:
        response = get_model_properties(model_name)
        if response.success:
            time.sleep(3.0)
            os.execv(command[0], command)
    except rospy.ServiceException:
        pass
    time.sleep(1.0)

raise SystemExit("ROS shutdown while waiting for model {}".format(model_name))
