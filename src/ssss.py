#! /usr/bin/env python3

import roslib
import rospy
import actionlib
import tf
from nav_msgs.msg import Odometry

class DockingMoveTo:
    ldjfslasdg
    lksjdflkasjf

    def calc_cmd(pose, marker_pose):
        return cmd;

class DockingServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(self._action_name, move_base_msgs.msg.MoveBaseAction, execute_cb=self.execute_cb, auto_start = False)
        self.odom_pose_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        self.odom_pose = Odometry()
        self.trans= []
        self.rot= []

        self.server.start()

        self.docking_calc = DockingMoveTo();

    def odom_callback(self, msg):
        self.odom_pose = msg

        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform('/odom','/safe_link',rospy.Time(0), rospy.Duration(4.0))
                (self.trans, self.rot) = self.listener.lookupTransform("/odom", "/safe_link", rospy.Time(0))
            except:
                print("tf listener ddong")
        # bagfile run and confirm sub_odom

    def execute(self, goal):
        # Do lots of awesome groundbreaking robot stuff here
        cmd = self.docking_calc(odom_pose, marker_pose)
        
        if not is_goal:
            publish(cmd)
        else:
            goal_reach



            self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('do_dishes_server')
  server = DockingServer(rospy.get_name())
  rospy.spin()