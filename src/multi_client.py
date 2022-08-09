#! /usr/bin/env python3

from lzma import MODE_FAST
import actionlib
from actionlib_msgs.msg import GoalStatus
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

            result = self.client.get_result()
            if result:
                state = self.client.get_state()

                print('get statae:', state)

                if state == GoalStatus.SUCCEEDED:
                    # print("%dth MoveBase_Action Complete!!"% (self.cnt + 1))
                    print("th MoveBase_Action Complete!!")
                    self.move_reach_goal = True
                else:
                    print("Action Failed")
                    self.move_reach_goal = False

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

        self.move_base_client = MoveBaseClient()
        self.go_to_docking = DockingClient()
        
        self._is_reach_multi_goal = False
        
        # input robot's goal position x,y & orientation z,w in first_goal_list and second_goal_list
        self.move_base_list = []
        self.room_goal_list  = [-5.540212631225586, -14.593623161315918, -0.9993664285511984, 0.03559131192893247]
        self.toilet_goal_list = [-1.7892355918884277, 8.190496444702148, 0.7011186191447972, 0.7130446563073681]
        self.lab_goal_list = [1.3657242059707642, 0.003745555877685547 , -0.002370253081383217,0.9999971909462197 ]

        self.move_base_list.append(self.room_goal_list)
        # self.move_base_list.append(self.toilet_goal_list)
        self.move_base_list.append(self.lab_goal_list)
        # print(len(self.move_base_list))
        print(self.move_base_list[1])

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

        if self.move_base_client.move_to_goal(self.xyzw_to_mbgoal(self.move_base_list[0])):
        
            self.go_to_docking.move_to_safe()
            self.go_to_docking.docking_reach_goal = False
            print("====" * 3 + " 1th docking action complete " + "====" *3)
            print("")

        self.move_base_client.move_reach_goal = False

        if self.move_base_client.move_to_goal(self.xyzw_to_mbgoal(self.move_base_list[1])):
        
            self.go_to_docking.move_to_safe()
            self.go_to_docking.docking_reach_goal = False
            print("====" * 3 + " 1th docking action complete " + "====" *3)
            print("")
        
        self.move_base_client.move_reach_goal = False

        if self.move_base_client.move_to_goal(self.xyzw_to_mbgoal(self.move_base_list[0])):
        
            self.go_to_docking.move_to_safe()
            self.go_to_docking.docking_reach_goal = False
            print("====" * 3 + " 1th docking action complete " + "====" *3)
            print("")
        
        self.move_base_client.move_reach_goal = False

        if self.move_base_client.move_to_goal(self.xyzw_to_mbgoal(self.move_base_list[1])):
        
            self.go_to_docking.move_to_safe()
            self.go_to_docking.docking_reach_goal = False
            print("====" * 3 + " 1th docking action complete " + "====" *3)
            print("")
        
        self.move_base_client.move_reach_goal = False
        
if __name__ == '__main__':
    try:
        rospy.init_node('Docking_server_client')
        result = MissionClient()
        result.run_mission()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
