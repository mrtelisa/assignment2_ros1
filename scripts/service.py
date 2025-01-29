#! /usr/bin/env python3

import rospy 
import sys

# Importing the service message and response type from the custom ROS package
from ass2_ros1.srv import Target, TargetResponse
from ass2_ros1.srv import SetLinVel, SetLinVelResponse
form geometry_msgs.msg import Twist

def read_target(req): pass 

def set_linvel(req): pass

service1 = rospy.Service("/target_service", Target, read_target)
service_vel = rospy.Service("/linvel_service", SetLinVel, set_linvel)

def set_linvel(req):
    vel_des = Twist()
    vel_des.linear.x = req.desx
    vel_des.linear.y = req.desy

    vel_pub.publish(vel_des)
    rospy.loginfo("Linear velocity set correctly to: x = %f, y = %f.", req.desx, req.desy)

    return SetLinVelResponse(success = True)


# Function for the service callback     
def read_target(req):
    target_x = float(rospy.get_param("/des_pos_x"))
    target_y = float(rospy.get_param("/des_pos_y"))

    return TargetResponse(target_x, target_y)

if __name__ == "__main__":
    try:
        rospy.init_node("service_node")

        vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        print("Service node interrpted", file = sys.stderr)