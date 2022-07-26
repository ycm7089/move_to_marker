#! /usr/bin/env python3

import roslib
import rospy
import actionlib
import tf
import move_base_msgs.msg
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped

class DockingMoveTo:
    def __init__(self):
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        
        self.robot_vel = Twist()
        self.is_reach_goal = False

    def move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal):
        self.is_reach_goal = False

        x = x_start
        y = y_start
        theta = theta_start

        x_diff = x_goal - x
        y_diff = y_goal - y

        x_traj, y_traj = [], []

        rho = np.hypot(x_diff, y_diff)

        if rho > 0.15:
            x_traj.append(x)
            y_traj.append(y)
            
            x_diff = x_goal - x
            y_diff = y_goal - y

            rho, v, w = controller.calc_control_command(
                x_diff, y_diff, theta, theta_goal)
            
            # print(rho)

            if abs(v) > MAX_LINEAR_SPEED:
                v = MAX_LINEAR_SPEED
                
            if abs(w) > MAX_ANGULAR_SPEED:
                w = np.sign(w) * MAX_ANGULAR_SPEED

        else :
            # theta_goal = goal
            # theta = robot
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
                self.is_reach_goal = True

        print("speed (v : %.2f w : %.6f) distance_to_marker(x : %.2f y : %.2f total %.2f)"% (v, w * 180.0 / math.pi, x_diff, y_diff, rho))
        cal_cmd(v,w)

        return self.is_reach_goal
        
    def cal_cmd(self, v, w) :

        self.robot_vel.linear.x = v 
        self.robot_vel.linear.y = 0.0
        self.robot_vel.linear.z = 0.0
        
        self.robot_vel.angular.x = 0.0
        self.robot_vel.angular.y = 0.0
        self.robot_vel.angular.z = w

        self.vel_pub.publish(robot_vel)


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
        print("SS")
        self._action_name = name
        print(self._action_name)
        self.server = actionlib.SimpleActionServer(self._action_name, move_base_msgs.msg.MoveBaseAction, execute_cb=self.execute_cb, auto_start = False)
        
        self.odom_pose_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.odom_pose = Odometry()
        self.Docking_goal = PoseStamped()

        self._to_goal_trans= []
        self._to_goal_rot= []

        self._reach_Docking_goal = False
        self._marker_visible = False

        self.server.start()

    def euler_from_quaternion(x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z # in radians

    def odom_callback(self, msg):
        print("okay")
        self.odom_pose = msg

        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform('/odom','/safe_link',rospy.Time(0), rospy.Duration(4.0))
                (self._to_goal_trans, self._to_goal_rot) = self.listener.lookupTransform("/odom", "/safe_link", rospy.Time(0))
                
                self._marker_visible = True
                
                print(self._to_goal_trans)
            except:
                print("tf listener ddong")
        # bagfile run and confirm sub_odom

    def execute_cb(self, Docking_goal):
        # cmd = self.docking_calc(odom_pose, marker_pose)
        
        # if not is_goal:
        #     publish(cmd)
        # else:
        #     goal_reach
        if self._marker_visible :
            if not self._reach_Docking_goal :

                to_goal_roll, to_goal_pitch, to_goal_yaw = self.euler_from_quaternion(_to_goal_rot[0], _to_goal_rot[1], _to_goal_rot[2], _to_goal_rot[3])

                Docking_goal.pose.position.x = self._to_goal_trans[0]
                Docking_goal.pose.position.y = self._to_goal_trans[1]
                Docking_goal.pose.position.z = 0.0

                Docking_goal.pose.orientation.x = 0.0
                Docking_goal.pose.orientation.y = 0.0
                Docking_goal.pose.orientation.z = self._to_goal_rot[2]
                Docking_goal.pose.orientation.w = self._to_goal_rot[3]

                odom_roll, odom_pitch, odom_yaw = self.euler_from_quaternion(self.odom_pose.pose.orientation.x, self.odom_pose.pose.orientation.y, self.odom_pose.pose.orientation.z, self.odom_pose.pose.orientation.w)

                x_start = self.odom_pose.pose.position.x
                y_start = self.odom_pose.pose.position.y
                theta_start = odom_yaw #radian

                x_goal = Docking_goal.pose.pose.position.x
                y_goal = Docking_goal.pose.pose.position.y
                theta_goal = to_goal_yaw #radian

                self._reach_Docking_goal = DockingMoveTo.move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)
            
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