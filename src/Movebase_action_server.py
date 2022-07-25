import rospy

import actionlib

import move_base_msgs.msg
from nav_msgs.msg import Odometry

class MoveBaseAction(object) :
    _feedback = move_base_msgs.msg.MoveBaseFeedback()
    _result = move_base_msgs.msg.MoveBaseResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, move_base_msgs.msg.MoveBaseAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self.odom_pose_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def odom_callback(self, msg):
        self.odom_pose = msg

    def execute_cb(self, goal, odom_pose):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # current robot pose => feedback
        self._feedback.base_position.pose.position.x = self.odom_pose.pose.postion.x
        self._feedback.base_position.pose.position.y = self.odom_pose.pose.postion.y

        # # start executing the action
        # current robot pose = goal_pose
        for i in range(1, int(goal.target_pose.pose.position.x)):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            for j in 
            self._feedback.base_position.pose.position.x = self._feedback.base_position.pose.position.x + 1.0
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
          
        if success:
            self._result = self._feedback.base_position.pose.position.x
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('Docking_server')
    server = MoveBaseAction(rospy.get_name())
    rospy.spin()
