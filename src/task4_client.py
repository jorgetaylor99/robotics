#!/usr/bin/env python3

import rospy
from com2009_msgs.srv import SetBool, SetBoolRequest
import sys


service_name = "identify_colour_service"

rospy.init_node(f"{service_name}_client")

rospy.wait_for_service(service_name)

service = rospy.ServiceProxy(service_name, SetBool)

service_request = SetBoolRequest()
service_request.request_signal = True

service_response = service(service_request)
print("Here is service_response:",service_response)