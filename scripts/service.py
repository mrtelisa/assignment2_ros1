#! /usr/bin/env python3

import rospy 
import sys

# Importing the service message and response type from the custom ROS package
from ass2_ros1.srv import Target, TargetResponse

def read_target(req): pass 

service = rospy.Service("/target_service", Target, read_target)

# Function for the service callback     
def read_target(req):
    target_x = float(rospy.get_param("/des_pos_x"))
    target_y = float(rospy.get_param("/des_pos_y"))

    return TargetResponse(target_x, target_y)

if __name__ == "__main__":
    try:
        rospy.init_node("service_node")

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        print("Service node interrpted", file = sys.stderr)