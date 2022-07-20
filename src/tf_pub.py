#!/usr/bin/env python3

import rospy
import math
import tf

def tf_pub():
    br = tf.TransformBroadcaster()

    br.sendTransform((0.3, 0.0, 0.2), tf.transformations.quaternion_from_euler(-90.0*math.pi/180.0, 0.0, -90.0*math.pi/180.0), rospy.Time.now(), "camera_0", "base_link")

    br.sendTransform((0.0, 0.0, 0.75), tf.transformations.quaternion_from_euler(0.0, 90.0*math.pi/180.0, 0.0), rospy.Time.now(), "safe_link", "marker")

def main():
    rospy.init_node('tf_publisher')
    while not rospy.is_shutdown():
        tf_pub()
    


if __name__ == '__main__':
    main()
