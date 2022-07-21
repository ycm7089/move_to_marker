#!/usr/bin/env python3

import numpy as np
import actionlib
import rospy
import math
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import tf

class RosClient:
    def __init__(self):
        self.odom_pose_sub = rospy.Subscriber("/odom",Odometry, self.odom_callback)

        self.odom_pose = Odometry()
        self.listener = tf.TransformListener()
        self.trans= []
        self.rot= []

        self.is_reach_goal = False
        
    def odom_callback(self, msg):
        self.odom_pose = msg

    def tf_listener(self):
        
        while not rospy.is_shutdown():
            self.listener.waitForTransform('/odom','/safe_link',rospy.Time(0), rospy.Duration(4.0))
            (self.trans, self.rot) = self.listener.lookupTransform("/odom", "/safe_link", rospy.Time(0))
            #trans :  xyz, rot : xyzw  

            if not self.is_reach_goal:
                self.is_reach_goal = move_toward_marker(self.trans,self.rot, self.odom_pose.pose)
            else:
                print("Gooooooooooooal!!")

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
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

def move_toward_marker(trans, rot, current_robot_pose):

    move_base_goal = MoveBaseGoal()
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    
    move_base_goal.target_pose.header.frame_id = "map"
    move_base_goal.target_pose.header.stamp = rospy.Time.now()

    move_base_goal.target_pose.pose.orientation.x = 0
    move_base_goal.target_pose.pose.orientation.y = 0
    move_base_goal.target_pose.pose.orientation.z = rot[2]
    move_base_goal.target_pose.pose.orientation.w = rot[3]

    move_base_goal.target_pose.pose.position.x = trans[0] - 0.3
    move_base_goal.target_pose.pose.position.y = trans[1]
    move_base_goal.target_pose.pose.position.z = 0

    client.send_goal(move_base_goal) 
    wait = client.wait_for_result()

    if not wait:
        print("gogogogo")

    else :
        goal = MoveBaseGoal()

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
        
        odom_info = Odometry()
        odom_info = current_robot_pose

        r, p, y = euler_from_quaternion(odom_info.pose.orientation.x, odom_info.pose.orientation.y, odom_info.pose.orientation.z, odom_info.pose.orientation.w)

        x_start = odom_info.pose.position.x
        y_start = odom_info.pose.position.y
        theta_start = y #radian

        x_goal = goal.target_pose.pose.position.x
        y_goal = goal.target_pose.pose.position.y
        theta_goal = yaw #radian

        is_reach_goal = move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)

        return is_reach_goal

class PathFinderController:
    """
    Constructs an instantiate of the PathFinderController for navigating a
    3-DOF wheeled robot on a 2D plane

    Parameters
    ----------
    Kp_rho : The linear velocity gain to translate the robot along a line
             towards the goal
    Kp_alpha : The angular velocity gain to rotate the robot towards the goal
    Kp_beta : The offset angular velocity gain accounting for smooth merging to
              the goal angle (i.e., it helps the robot heading to be parallel
              to the target angle.)
    """

    def __init__(self, Kp_rho, Kp_alpha, Kp_beta):
        self.Kp_rho = Kp_rho
        self.Kp_alpha = Kp_alpha
        self.Kp_beta = Kp_beta

    def calc_control_command(self, x_diff, y_diff, theta, theta_goal):
        """
        Returns the control command for the linear and angular velocities as
        well as the distance to goal

        Parameters
        ----------
        x_diff : The position of target with respect to current robot position
                 in x direction
        y_diff : The position of target with respect to current robot position
                 in y direction
        theta : The current heading angle of robot with respect to x axis
        theta_goal: The target angle of robot with respect to x axis

        Returns
        -------
        rho : The distance between the robot and the goal position
        v : Command linear velocity
        w : Command angular velocity
        """

        # Description of local variables:
        # - alpha is the angle to the goal relative to the heading of the robot
        # - beta is the angle between the robot's position and the goal
        #   position plus the goal angle
        # - Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards
        #   the goal
        # - Kp_beta*beta rotates the line so that it is parallel to the goal
        #   angle
        #
        # Note:
        # we restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn

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

# simulation parameters
controller = PathFinderController(1, 1, 1)

# Robot specifications
MAX_LINEAR_SPEED = 0.15
MAX_ANGULAR_SPEED = 0.15

def move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal):
    is_reach_goal = False

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
            is_reach_goal = True

    print("speed (v : %.2f w : %.6f) distance_to_marker(x : %.2f y : %.2f total %.2f)"% (v, w * 180.0 / math.pi, x_diff, y_diff, rho))
    
    robot_vel = Twist()

    robot_vel.linear.x = v 
    robot_vel.linear.y = 0.0
    robot_vel.linear.z = 0.0

    robot_vel.angular.x = 0.0
    robot_vel.angular.y = 0.0
    robot_vel.angular.z = w
    
    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    vel_pub.publish(robot_vel)

    return is_reach_goal        

def main():
    rospy.init_node('move_to_pose')
    
    ros_client = RosClient()
    ros_client.tf_listener()    

if __name__ == '__main__':
    main()
