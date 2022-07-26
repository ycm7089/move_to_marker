#! /usr/bin/env python3

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import math
from nav_msgs.msg import Odometry

class DockingClient:
    def __init__(self):
        
        # publihser
        self.odom_pose_sub = rospy.Subscriber("/odom",Odometry, self.odom_callback)
        # action server
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.client.wait_for_server()
        
        self.docking_client = actionlib.SimpleActionClient('Docking_server', move_base_msgs.msg.MoveBaseAction)
        self.docking_client.wait_for_server()

        # action server (docking)
        # self.docking_client = action ~~~~~

        print("dalkjgndfjdfkgkn")

        self.odom_pose = Odometry()

        self.trans= []
        self.rot= []

        self.is_reach_goal = False
        self.is_move_reach_goal = False
        
    def odom_callback(self, msg):
        self.odom_pose = msg

    def tf_listener(self):
        
        while not rospy.is_shutdown():
            if not self.is_move_reach_goal:
                move_base_goal = MoveBaseGoal()
                roll, pitch, yaw = euler_from_quaternion(self.map_rot[0], self.map_rot[1], self.map_rot[2], self.map_rot[3])

                move_base_goal.target_pose.header.frame_id = "map"
                move_base_goal.target_pose.header.stamp = rospy.Time.now()                 

                move_base_goal.target_pose.pose.orientation.x = 0
                move_base_goal.target_pose.pose.orientation.y = 0
                # move_base_goal.target_pose.pose.orientation.z = -0.6580712166829297
                # move_base_goal.target_pose.pose.orientation.w = 0.7529556917730609
                # move_base_goal.target_pose.pose.position.x = 1.2797787189483643
                # move_base_goal.target_pose.pose.position.y = 0.07122102379798889
                move_base_goal.target_pose.pose.orientation.z = self.map_rot[2]
                move_base_goal.target_pose.pose.orientation.w = self.map_rot[3]
                move_base_goal.target_pose.pose.position.x = self.map_trans[0]
                move_base_goal.target_pose.pose.position.y = self.map_trans[1] + 0.3
                move_base_goal.target_pose.pose.position.z = 0

                # print("yaw is : %.3f"% (yaw * 180.0 / math.pi))
                # print(self.map_trans[0], self.map_trans[1], self.map_rot[2])
                
                self.client.send_goal(move_base_goal) 

                self.client.wait_for_result()

                if self.client.get_result():
                    print("YES")
                    self.is_move_reach_goal = True

                    # self.docking_client.send_goal(start)
                    # self.docking_client.wait_for_result()
                    # self.docking_client.get_result()  docking complete, marker univisible, 

            else :

                goal = MoveBaseGoal()
                print("sssssssssss")
                roll, pitch, yaw = euler_from_quaternion(rot[0], rot[1], rot[2], rot[3])

                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                
                goal.target_pose.pose.orientation.x = 0
                goal.target_pose.pose.orientation.y = 0
                goal.target_pose.pose.orientation.z = rot[2]
                goal.target_pose.pose.orientation.w = rot[3]

                goal.target_pose.pose.position.x = trans[0]
                goal.target_pose.pose.position.y = trans[1]
                goal.target_pose.pose.position.z = 0        
                
                self.is_reach_goal = move_toward_marker(self.trans,self.rot, self.odom_pose.pose, self.client)



if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('Docking_server_client')
        result = DockingClient()
        # print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
