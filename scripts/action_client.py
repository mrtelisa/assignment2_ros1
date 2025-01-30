#! /usr/bin/env python3

import rospy 
import actionlib.msg
import actionlib 
import sys

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import assignment_2_2024.msg
from ass2_ros1.msg import RobotPosVel, ActualVel, Goals

goals_reached = 0
goals_cancelled = 0

def pub_PosVel(msg): pass
def communicate_vel_km(mgs): pass

state_pub = rospy.Publisher("/robot_state", RobotPosVel, queue_size = 10)
actual_vel = rospy.Publisher("/actual_velocity", ActualVel, queue_size = 10)
rospy.Subscriber("/odom", Odometry, pub_PosVel)
rospy.Subscriber("/odom", Odometry, communicate_vel_km)

current_feedback = None
# Function to update the feedback 
def update_feedback(fd):
    global current_feedback
    current_feedback = fd

# Function to define a goal and sending it to the action server
def define_goal(client, des_x, des_y):
    goal = assignment_2_2024.msg.PlanningGoal()
    goal.target_pose = PoseStamped()
    goal.target_pose.pose.position.x = des_x
    goal.target_pose.pose.position.y = des_y
    rospy.loginfo("Goal defined")

    client.send_goal(goal, feedback_cb = update_feedback)
    rospy.loginfo("Goal sent")

def communicate_vel_km(msg):
    RobVel = ActualVel()
    RobVel.act_vx = msg.twist.twist.linear.x * 3.6
    RobVel.act_vy = msg.twist.twist.linear.y * 3.6

    actual_vel.publish(RobVel) 

# Function to make the user interact with the system while it is working
def interactions(client):
    user_request = input("Press: 'q' to cancel the goal; 'f' to recive feedback; 'e' to exit  ->  ")

    # The user wants to quit 
    if user_request.lower() == 'q':
        rospy.loginfo("Cancelling the goal previously defined")
        goals_cancelled += 1
        client.cancel_goal()

    # The user wants a feedback form the robot
    elif user_request.lower() == 'f':
        if current_feedback is None:
            rospy.loginfp("No feedback has been recieved, sorry!")
        else:
            rospy.loginfo("Latest feedback: %s", current_feedback)
        
    # The user wants to exit from the simulation
    elif user_request.lower() == 'e':
        rospy.loginfo("Cancelling the goal previously defined and exiting the simulation")        
        goals_cancelled += 1
        client.cancel_goal()
        return "exit"
    
    # In case the imput is not valid
    else:
        rospy.loginfo("Invalid input. Please retry!")

# Function to set the target from users's input
def define_target(str):
    while(1):
        try:
            coord = int(input(str))
            break
        except ValueError:
            print("Invalid. Please retry!")
    return coord

# Function to publish the position and the velocities of the robot
def pub_PosVel(msg):
    PosVel = RobotPosVel()
    PosVel.x = msg.pose.pose.position.x
    PosVel.y = msg.pose.pose.position.y
    PosVel.vel_x = msg.twist.twist.linear.x
    PosVel.vel_z = msg.twist.twist.linear.z

    state_pub.publish(PosVel)


if __name__ == '__main__':
    try:
        # Initialization of the rospy node
        rospy.init_node("action_client_node")
        rospy.sleep(2)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Defining the target and its desired coordinates
            des_x = define_target("Enter the desired x coordinate of the target:")
            if des_x == "exit": break
        
            des_y = define_target("Enter the desired y coordinate of the target:")
            if des_y == "exit": break        

            # Defining the client
            client = actionlib.SimpleActionClient('reaching_goal', assignment_2_2024.msg.PlanningAction)
            client.wait_for_server()
            
            # Officially setting the target
            define_goal(client, des_x, des_y)

            # Making the code run 
            while not rospy.is_shutdown():
                state = client.get_state()

                if state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal reached succesfully!")
                    goals_reached += 1
                    break
                
                if state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.PREEMPTED]:
                    rospy.loginfo("Goal not reached. Retry!")
                    break
            
                inter = interactions(client)
                if inter == "exit": exit()
                rate.sleep()
                
        rospy.loginfo("Exit succesfully")

    except rospy.ROSInterruptException:
        print("Action_client node interrupted", file = sys.stderr)

