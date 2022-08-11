#!/usr/bin/env python3  
import roslib
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((-0.5, -1.0, -1.5),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "base_link",
                         "base_camera")
        rate.sleep()