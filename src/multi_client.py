#! /usr/bin/env python3

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import math

# if Aruco marker is a lot, this code will need Aruco ID 

class MoveBaseClient:
    def __init__(self):

        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.client.wait_for_server()
        
        self.move_reach_goal = False

        self.move_base_goal = MoveBaseGoal()
        self.go_to_docking = DockingClient()
        
        # input robot's goal position x,y & orientation z,w in first_goal_list and second_goal_list
        self.move_base_list = []
        self.first_goal_list  = [1.0049941539764404, -0.005708754062652588, -0.9999863695455604, 0.005221180239162203]
        self.second_goal_list = [2.8684380054473877, 0.48713570833206177 , 0.11430083532461724, 0.9934461832651503]

        self.move_base_list.append(self.first_goal_list)
        self.move_base_list.append(self.second_goal_list)

        self.cnt = 0

    def move_to_goal(self) :
        
        if not self.move_reach_goal:
            print(" %dth MoveBase action start" % (self.cnt + 1)) 

            self.move_base_goal.target_pose.header.frame_id = "map"
            self.move_base_goal.target_pose.header.stamp = rospy.Time.now()                 

            self.move_base_goal.target_pose.pose.position.x = self.move_base_list[self.cnt][0]
            self.move_base_goal.target_pose.pose.position.y = self.move_base_list[self.cnt][1]
            self.move_base_goal.target_pose.pose.position.z = 0.0
            
            self.move_base_goal.target_pose.pose.orientation.x = 0.0
            self.move_base_goal.target_pose.pose.orientation.y = 0.0
            self.move_base_goal.target_pose.pose.orientation.z = self.move_base_list[self.cnt][2]
            self.move_base_goal.target_pose.pose.orientation.w = self.move_base_list[self.cnt][3]
            
            self.client.send_goal(self.move_base_goal) 

            self.client.wait_for_result()

            if self.client.get_result():
                print("%dth MoveBase_Action Complete!!"% (self.cnt + 1))
                
                self.move_reach_goal = True

                self.go_to_docking.move_to_safe()

                self.cnt = self.cnt + 1 
    
class DockingClient:
    def __init__(self):

        self.docking_client = actionlib.SimpleActionClient('/Docking_server', MoveBaseAction)
        self.docking_client.wait_for_server()

        self.docking_goal = MoveBaseGoal()
        
        self.docking_reach_goal = False

    def move_to_safe(self):
        print("Docking action start")
        
        self.docking_reach_goal = False

        self.docking_goal.target_pose.header.frame_id = "map"
        self.docking_goal.target_pose.header.stamp = rospy.Time.now()
        
        self.docking_goal.target_pose.pose.position.x = 0.0
        self.docking_goal.target_pose.pose.position.y = 0.0
        self.docking_goal.target_pose.pose.position.z = 0.0   
        
        self.docking_goal.target_pose.pose.orientation.x = 0.0
        self.docking_goal.target_pose.pose.orientation.y = 0.0
        self.docking_goal.target_pose.pose.orientation.z = 0.0
        self.docking_goal.target_pose.pose.orientation.w = 0.0     
        
        self.docking_client.send_goal(self.docking_goal)

        self.docking_client.wait_for_result()

        if self.docking_client.get_result():
            print("Docking Complete!!")
            self.docking_reach_goal = True

        return self.docking_reach_goal

class MultiClient :
    def __init__(self) :

        self.move_base_result = MoveBaseClient()
        self.docking_result = DockingClient()
        
        self._is_reach_multi_goal = False

    def Cycle(self):

        if not self._is_reach_multi_goal :
            self.move_base_result.move_to_goal()

            # when below code is True, it will perform
            if self.docking_result.move_to_safe() :

                self.move_base_result.move_reach_goal()
                print(self.move_base_result.move_reach_goal())

                self._is_reach_multi_goal = True
            else :
                print("Docking isn't complete")

        else :
            print("Multi Cycle Complete!!")           

if __name__ == '__main__':
    try:
        rospy.init_node('Docking_server_client')
        result = MultiClient()
        result.Cycle()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
