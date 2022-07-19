#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import actionlib
import rospy
import math
from actionlib_msgs.msg import *
from cm_aruco_msgs.msg import Aruco_marker
from geometry_msgs.msg import Pose, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import tf

from random import random

# current_robot_pose = Odometry()
# current_marker_pose = Pose()

class topic_sub:
    def __init__(self):
        self.robot_pose_sub = rospy.Subscriber("/odom",Odometry, self.odom_callback)
        self.marker_pose_sub = rospy.Subscriber("/aruco_poses", Aruco_marker, self.marker_callback) #marker pose
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        
    def odom_callback(self, msg):
        self.current_robot_pose = msg

    def marker_callback(self, msg):
        self.current_marker_pose = msg

def tf_listener():
    listener = tf.TransformListener()

    while not rospy.is_shutdown():
        listener.waitForTransform('/base_link','/safe_link',rospy.Time(0), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform("/base_link", "/safe_link", rospy.Time(0))

        #trans :  xyz, rot : xyzw  
        # print(trans[0])
        move_toward_marker(trans,rot)

        # quaternion check and Euler check   ===> rot is quaternion

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

def move_toward_marker(trans, rot):
    goal = MoveBaseGoal()

    roll, pitch, yaw = euler_from_quaternion(rot[0], rot[1], rot[2], rot[3])

    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = rot[2]
    goal.target_pose.pose.orientation.w = rot[3]
    
    goal.target_pose.pose.position.x = trans[0]
    goal.target_pose.pose.position.y = trans[1]
    goal.target_pose.pose.position.z = 0
    # print(trans[0])
    
    ac.send_goal(goal) 

    odom_info = topic_sub()
    r, p, y = euler_from_quaternion(odom_info.current_robot_pose.orientation.x, odom_info.current_robot_pose.orientation.y, odom_info.current_robot_pose.orientation.z, odom_info.current_robot_pose.orientation.w)

    x_start = odom_info.current_robot_pose.pose.position.x
    y_start = odom_info.current_robot_pose.pose.position.y
    theta_start = 2 * np.pi * y - np.pi #odom yaw radian
    
    x_goal = goal.target_pose.pose.position.x
    y_goal = goal.target_pose.pose.position.y
    theta_goal = 2 * np.pi * yaw - np.pi # goal yaw radian

    print("Initial x: %.2f m\nInitial y: %.2f m\nInitial theta: %.2f rad\n" %
            (x_start, y_start, theta_start))
    print("Goal x: %.2f m\nGoal y: %.2f m\nGoal theta: %.2f rad\n" %
            (x_goal, y_goal, theta_goal))

    move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)

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

        return rho, v, w


# simulation parameters
controller = PathFinderController(9, 15, 3)
dt = 0.01

# Robot specifications
MAX_LINEAR_SPEED = 15
MAX_ANGULAR_SPEED = 7

show_animation = True


def move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal):
    x = x_start
    y = y_start
    theta = theta_start

    x_diff = x_goal - x
    y_diff = y_goal - y

    x_traj, y_traj = [], []

    rho = np.hypot(x_diff, y_diff)
    while rho > 0.001:
        x_traj.append(x)
        y_traj.append(y)

        x_diff = x_goal - x
        y_diff = y_goal - y

        rho, v, w = controller.calc_control_command(
            x_diff, y_diff, theta, theta_goal)

        if abs(v) > MAX_LINEAR_SPEED:
            v = np.sign(v) * MAX_LINEAR_SPEED

        if abs(w) > MAX_ANGULAR_SPEED:
            w = np.sign(w) * MAX_ANGULAR_SPEED

        theta = theta + w * dt
        robot_vel = Twist()
        
        robot_vel.linear.x = v * np.cos(theta)
        robot_vel.linear.y = v * np.sin(theta)
        robot_vel.linear.z = 0.0

        robot_vel.angular.x = 0.0
        robot_vel.angular.y = 0.0
        robot_vel.angular.z = w
        robot_vel.angular.w = 1.0

        vel_pub.publish(robot_vel)

        x = x + v * np.cos(theta) * dt
        y = y + v * np.sin(theta) * dt

        if show_animation:  # pragma: no cover
            plt.cla()
            plt.arrow(x_start, y_start, np.cos(theta_start),
                      np.sin(theta_start), color='r', width=0.1)
            plt.arrow(x_goal, y_goal, np.cos(theta_goal),
                      np.sin(theta_goal), color='g', width=0.1)
            plot_vehicle(x, y, theta, x_traj, y_traj)


def plot_vehicle(x, y, theta, x_traj, y_traj):  # pragma: no cover
    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = transformation_matrix(x, y, theta)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

    plt.plot(x_traj, y_traj, 'b--')

    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])

    plt.xlim(0, 20)
    plt.ylim(0, 20)

    plt.pause(dt)


def transformation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])


def main():
    rospy.init_node('move_to_pose')
    # while not rospy.is_shutdown():
    topic_sub()
    tf_listener()

if __name__ == '__main__':
    main()
