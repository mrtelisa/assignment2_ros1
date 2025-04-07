#! /usr/bin/env python3

"""
This node implements an action client that allows the user to set a target position (des_x, des_y), cancel the current goal, 
or receive feedback from the action server. Additionally, it publishes the robot's current position and velocity as a custom message 
(x, y, vel_x, vel_z) based on the data from the /odom topic.

.. module:: action_client_node
   :platform: Unix
   :synopsis: Action client node for sending goals, handling user interactions, and publishing robot state.

.. moduleauthor:: Elisa Martinenghi <s6504193@studenti.unige.it>
"""


import rospy 
import actionlib.msg
import actionlib 
import sys

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import assignment_2_2024.msg
from ass2_ros1.msg import RobotPosVel

def pub_PosVel(msg): pass

state_pub = rospy.Publisher("/robot_state", RobotPosVel, queue_size=10)
rospy.Subscriber("/odom", Odometry, pub_PosVel)

current_feedback = None
# Function to update the feedback 
def update_feedback(fd):
    """
    Function to handle feedback from the action server.

    :param fd: The feedback message received from the action server.
    :type fd: PlanningFeedback
    """
    global current_feedback
    current_feedback = fd

# Function to define a goal and sending it to the action server
def define_goal(client, des_x, des_y):
    """
    Function to send a goal to the action server.

    :param client: The action client
    :type client: SimpleActionClient
    :param des_x: The x coordinate of the desired goal
    :type des_x: float
    :param des_y: The y coordinate of the desired goal
    :type des_y: float
    """
    goal = assignment_2_2024.msg.PlanningGoal()
    goal.target_pose = PoseStamped()
    goal.target_pose.pose.position.x = des_x
    goal.target_pose.pose.position.y = des_y
    rospy.loginfo("Goal defined")

    client.send_goal(goal, feedback_cb = update_feedback)
    rospy.loginfo("Goal sent")

# Function to make the user interact with the system while it is working
def interactions(client):
    """
    Function to handle user interaction during goal execution.
    Allows the user to cancel the goal, receive feedback, or exit.

    :param client: The action client
    :type client: SimpleActionClient

    :returns: 'exit' if the user chooses to exit, None otherwise
    :rtype: str or None
    """
    user_request = input("Press: 'q' to cancel the goal; 'f' to recive feedback; 'e' to exit  ->  ")

    # The user wants to quit 
    if user_request.lower() == 'q':
        rospy.loginfo("Cancelling the goal previously defined")
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
        client.cancel_goal()
        return "exit"
    
    # In case the imput is not valid
    else:
        rospy.loginfo("Invalid input. Please retry!")

# Function to set the target from users's input
def define_target(str):
    """
    Function to read a target coordinate from the user input and ensure it is valid.

    :param prompt: Prompt to show to the user
    :type prompt: str

    :returns: Validated coordinate as float
    :rtype: float
    """
    while(1):
        try:
            coord = int(input(str))
            break
        except ValueError:
            print("Invalid. Please retry!")
    return coord

# Function to publish the position and the velocities of the robot
def pub_PosVel(msg):
    """
    Function that publishes the robot position and velocity as a custom message (x, y, vel_x, vel_z) 
    by relying on the values published on the topic /odom.

    :param msg: The Odometry message from the /odom topic
    :type msg: nav_msgs.msg.Odometry
    """
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