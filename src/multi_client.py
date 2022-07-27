#! /usr/bin/env python3

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import math
from nav_msgs.msg import Odometry

class MoveBaseClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.client.wait_for_server()
        

        self.is_move_reach_goal = False

        self.go_to_docking = DockingClient()

    def move_to_goal(self) :
                   
        if not self.is_move_reach_goal:
            print("MoveBase action start")

            move_base_goal = MoveBaseGoal()

            move_base_goal.target_pose.header.frame_id = "map"
            move_base_goal.target_pose.header.stamp = rospy.Time.now()                 

            move_base_goal.target_pose.pose.position.x = 1.0049941539764404
            move_base_goal.target_pose.pose.position.y = -0.005708754062652588
            move_base_goal.target_pose.pose.position.z = 0
            
            move_base_goal.target_pose.pose.orientation.x = 0
            move_base_goal.target_pose.pose.orientation.y = 0
            move_base_goal.target_pose.pose.orientation.z = -0.9999863695455604
            move_base_goal.target_pose.pose.orientation.w = 0.005221180239162203

            self.client.send_goal(move_base_goal) 

            self.client.wait_for_result()

            if self.client.get_result():
                print("MoveBase_Action Complete!!")
                
                self.is_move_reach_goal = True
                self.go_to_docking.move_to_safe()


class DockingClient:
    def __init__(self):
        
        # subscriber
        self.odom_pose_sub = rospy.Subscriber("/odom",Odometry, self.odom_callback)

        # docking server
        self.docking_client = actionlib.SimpleActionClient('/Docking_server', MoveBaseAction)
        self.docking_client.wait_for_server()
      

        self.odom_pose = Odometry()

        self.trans= []
        self.rot= []

        self.is_reach_goal = False
        
    def odom_callback(self, msg):
        self.odom_pose = msg

    def move_to_safe(self):
        print("Docking action start")

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 0.0

        goal.target_pose.pose.position.x = 0.0
        goal.target_pose.pose.position.y = 0.0
        goal.target_pose.pose.position.z = 0        
        
        self.docking_client.send_goal(goal)

        self.docking_client.wait_for_result()

        if self.docking_client.get_result():
            print("Docking Complete!!")



if __name__ == '__main__':
    try:
        rospy.init_node('Docking_server_client')
        result = MoveBaseClient()
        result.move_to_goal()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
