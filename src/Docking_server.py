#! /usr/bin/env python3

from distutils.util import execute
from re import X
import roslib
import rospy
import actionlib
import tf
import move_base_msgs.msg
import math
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped

class DockingMoveTo:
    def __init__(self):
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        
        self.robot_vel = Twist()

    def move_to_pose(self, x_start, y_start, theta_start, x_goal, y_goal, theta_goal):
        is_reach_goal = False

        x = x_start
        y = y_start
        theta = theta_start

        x_diff = x_goal - x
        y_diff = y_goal - y
        theta_goal = theta_goal
        # value is not changed

        # print(x,y,x_diff,y_diff,x_goal,y_goal)
        
        rho = np.hypot(x_diff, y_diff)
        # print(rho)
        if rho > 0.15:
            
            x_diff = x_goal - x
            y_diff = y_goal - y

            rho, v, w = controller.calc_control_command(
                x_diff, y_diff, theta, theta_goal)
            
            if abs(v) > MAX_LINEAR_SPEED:
                v = MAX_LINEAR_SPEED
                
            if abs(w) > MAX_ANGULAR_SPEED:
                w = np.sign(w) * MAX_ANGULAR_SPEED
            # print("speed (v : %.2f w : %.6f degree) distance_to_marker(x : %.2f y : %.2f total %.2f)"% (v, w * 180.0 / math.pi, x_diff, y_diff, rho))
            # print(x_diff, y_diff, rho)
        else :
            # theta_goal = goal
            # theta = robot
            print("YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY")
            att = theta - theta_goal

            v = 0.0

            if att > 0:
                att = -(att)
                                        
            elif att < 0:
                att = abs(att)
            
            w = att

            if w > MAX_ANGULAR_SPEED:
                w = np.sign(w) * MAX_ANGULAR_SPEED

            elif abs(w) < 0.001 :
                w = 0.0
                is_reach_goal = True
        print("======" * 15)
        print("")
        # # print("speed (v : %.2f w : %.6f degree) distance_to_marker(x : %.2f y : %.2f total %.2f)"% (v, w * 180.0 / math.pi, x_diff, y_diff, rho))
        # print("")

        self.cal_cmd(v,w)

        return is_reach_goal
        
    def cal_cmd(self, v, w) :

        self.robot_vel.linear.x = v 
        self.robot_vel.linear.y = 0.0
        self.robot_vel.linear.z = 0.0
        
        self.robot_vel.angular.x = 0.0
        self.robot_vel.angular.y = 0.0
        self.robot_vel.angular.z = w

        self.vel_pub.publish(self.robot_vel)

class PathFinderController:
  
    def __init__(self, Kp_rho, Kp_alpha, Kp_beta):
        self.Kp_rho = Kp_rho
        self.Kp_alpha = Kp_alpha
        self.Kp_beta = Kp_beta

    def calc_control_command(self, x_diff, y_diff, theta, theta_goal):

        rho = np.hypot(x_diff, y_diff)

        alpha = (np.arctan2(y_diff, x_diff)
                 - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi
        v = self.Kp_rho * rho
        w = self.Kp_alpha * alpha - controller.Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v
        # print("rho : %.2f alpha : %.2f beta : %.2f linear vel : %.2f angular vel : %.2f" % (rho, alpha, beta, v, w))
        return rho, v, w

controller = PathFinderController(1, 1, 1)

MAX_LINEAR_SPEED = 0.15
MAX_ANGULAR_SPEED = 0.15

class DockingServer(object):
    def __init__(self, name):
        self._action_name = name
        print(self._action_name)
        self.odom_pose_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.server = actionlib.SimpleActionServer(self._action_name, move_base_msgs.msg.MoveBaseAction, execute_cb=self.execute_cb, auto_start = False)

        self.odom_pose = Odometry()
        self.listener = tf.TransformListener()

        self._to_goal_trans= []
        self._to_goal_rot= []

        self._reach_Docking_goal = False
        self._marker_visible = False

        self.move = DockingMoveTo()
        self.reached = False

        self.server.start()

    def euler_from_quaternion(self, x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        self.roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        self.pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        self.yaw_z = math.atan2(t3, t4)
        
        return self.roll_x, self.pitch_y, self.yaw_z # in radians

    def odom_callback(self, msg):
        print("okay")
        self.odom_pose = msg

        # while not rospy.is_shutdown():            
        try:
            self.listener.waitForTransform('/odom','/safe_link',rospy.Time(0), rospy.Duration(4.0))
            (self._to_goal_trans, self._to_goal_rot) = self.listener.lookupTransform("/odom", "/safe_link", rospy.Time(0))
            # print(self.odom_pose)

            self._marker_visible = True

        except:
            print("tf listener ddong")

    def execute_cb(self, Docking_goal):

        print(self._marker_visible)
        while not self._reach_Docking_goal:
            if self._marker_visible :
                if not self._reach_Docking_goal :
                    rospy.sleep(0.01)
                    to_goal_roll, to_goal_pitch, to_goal_yaw = self.euler_from_quaternion(self._to_goal_rot[0], self._to_goal_rot[1], self._to_goal_rot[2], self._to_goal_rot[3])

                    Docking_goal.target_pose.pose.position.x = self._to_goal_trans[0]
                    Docking_goal.target_pose.pose.position.y = self._to_goal_trans[1]
                    Docking_goal.target_pose.pose.position.z = 0.0
                    Docking_goal.target_pose.pose.orientation.x = 0.0
                    Docking_goal.target_pose.pose.orientation.y = 0.0
                    Docking_goal.target_pose.pose.orientation.z = self._to_goal_rot[2]
                    Docking_goal.target_pose.pose.orientation.w = self._to_goal_rot[3]

                    odom_roll, odom_pitch, odom_yaw = self.euler_from_quaternion(self.odom_pose.pose.pose.orientation.x, self.odom_pose.pose.pose.orientation.y, self.odom_pose.pose.pose.orientation.z, self.odom_pose.pose.pose.orientation.w)

                    x_start = self.odom_pose.pose.pose.position.x
                    y_start = self.odom_pose.pose.pose.position.y
                    theta_start = odom_yaw #radian

                    x_goal = Docking_goal.target_pose.pose.position.x
                    y_goal = Docking_goal.target_pose.pose.position.y
                    theta_goal = to_goal_yaw #radian

                    self._reach_Docking_goal = self.move.move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)

                else:
                    # if self.server.is_preempt_requested():
                    #     rospy.loginfo('%s: Preempted' % self._action_name)
                    #     self.server.set_preempted()
                    #     break
                    # else :
                    self.server.set_succeeded()
                    print('Success')
            else :
                print("I can't see the marker")

if __name__ == '__main__':
    rospy.init_node('Docking_server')

    print("입력해주세요")
    _input_go = input()

    if _input_go == "go" :
        print("Y")
        server = DockingServer(rospy.get_name())
        rospy.spin()

    else :
        print('잘못 입력하셨습니다.')