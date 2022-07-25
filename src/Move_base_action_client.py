#! /usr/bin/env python3

import rospy
# from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib
from nav_msgs.msg import Odometry


# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import move_base_msgs.msg

def docking_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('Docking_server', move_base_msgs.msg.MoveBaseAction)
    
    odom = Odometry()
    odom.pose.pose.position.y = 3.0
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    # for i in cm_list:
    goal = move_base_msgs.msg.MoveBaseGoal()
    goal.target_pose.pose.position.x = 5.0
    goal.target_pose.pose.position.y = odom.pose.pose.position.y

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    print(goal)
    print(client.get_result())
            
    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('Docking_server_client')
        result = docking_client()
        # print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)