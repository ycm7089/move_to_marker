#! /usr/bin/env python3

from lzma import MODE_FAST
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


    def move_to_goal(self, goal) :
        self.move_reach_goal = False
        if not self.move_reach_goal:
            # print(" %dth MoveBase action start" % (self.cnt + 1))
            print("MoveBase action start") 

            self.client.send_goal(goal) 

            self.client.wait_for_result()

            if self.client.get_result():
                # print("%dth MoveBase_Action Complete!!"% (self.cnt + 1))
                print("th MoveBase_Action Complete!!")
                
                self.move_reach_goal = True

            return self.move_reach_goal
    
class DockingClient:
    def __init__(self):

        self.docking_client = actionlib.SimpleActionClient('/Docking_server', MoveBaseAction)
        self.docking_client.wait_for_server()

        self.docking_goal = MoveBaseGoal()

        self.docking_goal.target_pose.header.frame_id = "map"
        self.docking_goal.target_pose.header.stamp = rospy.Time.now()

        self.docking_goal.target_pose.pose.position.x = 0.0
        self.docking_goal.target_pose.pose.position.y = 0.0
        self.docking_goal.target_pose.pose.position.z = 0.0   
        
        self.docking_goal.target_pose.pose.orientation.x = 0.0
        self.docking_goal.target_pose.pose.orientation.y = 0.0
        self.docking_goal.target_pose.pose.orientation.z = 0.0
        self.docking_goal.target_pose.pose.orientation.w = 0.0     
        
        self.docking_reach_goal = False

    def move_to_safe(self):
        self.docking_reach_goal = False
        
        if not self.docking_reach_goal:
            print("Docking action start")
            self.docking_goal.target_pose.header.stamp = rospy.Time.now()
                        
            self.docking_client.send_goal(self.docking_goal)

            self.docking_client.wait_for_result()

            if self.docking_client.get_result():
                print("Docking Complete!!")
                self.docking_reach_goal = True

        return self.docking_reach_goal

class MissionClient :
    def __init__(self) :

        self.move_base_result = MoveBaseClient()
        self.se_move_base_result = MoveBaseClient()
        # self.docking_result = DockingClient()
        
        self._is_reach_multi_goal = False

        self.go_to_docking = DockingClient()
        
        # input robot's goal position x,y & orientation z,w in first_goal_list and second_goal_list
        self.move_base_list = []
        self.first_goal_list  = [-5.540212631225586, -14.593623161315918, -0.9993664285511984, 0.03559131192893247]
        self.second_goal_list = [0.9514656662940979, 0.016252458095550537 , 0.03029387883189036, 0.9995410351282826]

        self.move_base_list.append(self.first_goal_list)
        self.move_base_list.append(self.second_goal_list)
        # print(len(self.move_base_list))

    def xyzw_to_mbgoal(self, xyzw):
        self.mbgoal = MoveBaseGoal()

        self.mbgoal.target_pose.header.frame_id = "map"
        self.mbgoal.target_pose.header.stamp = rospy.Time.now()

        self.mbgoal.target_pose.pose.position.x = xyzw[0]
        self.mbgoal.target_pose.pose.position.y = xyzw[1]
        self.mbgoal.target_pose.pose.position.z = 0.0   
        
        self.mbgoal.target_pose.pose.orientation.x = 0.0
        self.mbgoal.target_pose.pose.orientation.y = 0.0
        self.mbgoal.target_pose.pose.orientation.z = xyzw[2]
        self.mbgoal.target_pose.pose.orientation.w = xyzw[3]            

        return self.mbgoal

    def run_mission(self):

        self.move_base_result.move_to_goal(self.xyzw_to_mbgoal(self.move_base_list[0]))
        self.move_base_result.move_reach_goal = False
        print("====" * 3 + " 1th move_base action complete " + "====" *3)

        self.go_to_docking.move_to_safe()
        self.go_to_docking.docking_reach_goal = False
        print("====" * 3 + " 1th docking action complete " + "====" *3)
        print("")
        
        self.move_base_result.move_to_goal(self.xyzw_to_mbgoal(self.move_base_list[1])) 
        self.move_base_result.move_reach_goal = False
        print("====" * 3 + " 2th move_base action complete " + "====" *3)

        self.go_to_docking.move_to_safe()
        self.go_to_docking.docking_reach_goal = False
        print("====" * 3 + " 2th docking action complete " + "====" *3)
        print("")

        self.move_base_result.move_to_goal(self.xyzw_to_mbgoal(self.move_base_list[0]))
        self.move_base_result.move_reach_goal = False
        print("====" * 3 + " 3th move_base action complete " + "====" *3)

        self.go_to_docking.move_to_safe()
        self.go_to_docking.docking_reach_goal = False
        print("====" * 3 + " 3th docking action complete " + "====" *3)
        print("")

        self.move_base_result.move_to_goal(self.xyzw_to_mbgoal(self.move_base_list[1])) 
        self.move_base_result.move_reach_goal = False
        print("====" * 3 + " 4th move_base action complete " + "====" *3)

        self.go_to_docking.move_to_safe()
        self.go_to_docking.docking_reach_goal = False
        print("====" * 3 + " 4th docking action complete " + "====" *3)
        print("")

        self.move_base_result.move_to_goal(self.xyzw_to_mbgoal(self.move_base_list[0]))
        self.move_base_result.move_reach_goal = False
        print("====" * 3 + " 5th move_base action complete " + "====" *3)

        self.go_to_docking.move_to_safe()
        print("====" * 3 + " 5th docking action complete " + "====" *3)
        print("")
        
        print("====" * 3 + " All Actions Finished!! " + "====" * 3)      

if __name__ == '__main__':
    try:
        rospy.init_node('Docking_server_client')
        result = MissionClient()
        result.run_mission()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
