#! /usr/bin/env python3

"""
This node implements a ROS service that provides the desired target coordinates (x, y) by reading them from ROS parameters `/des_pos_x` and `/des_pos_y`.
The service can be called by other nodes to retrieve the current target coordinates.

.. module:: service_node
   :platform: Unix
   :synopsis: Service node to provide desired target coordinates from parameters.

.. moduleauthor:: Elisa Martinenghi <s6504193@studenti.unige.it>
"""

import rospy 
import sys

# Importing the service message and response type from the custom ROS package
from ass2_ros1.srv import Target, TargetResponse

def read_target(req): pass 

service = rospy.Service("/target_service", Target, read_target)

# Function for the service callback     
def read_target(req):
    """
    Service callback function that reads the target coordinates from ROS parameters
    and returns them in the response.

    :param req: The service request 
    :type req: ass2_ros1.srv.TargetRequest

    :returns: The response containing the target x and y coordinates
    :rtype: ass2_ros1.srv.TargetResponse
    """
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