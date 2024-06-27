#!/usr/bin/env python

import rospy
import tf

def get_robot_position():
    rospy.init_node('robot_position_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # Wait for the transformation between '/map' and '/base_link'
            listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(1.0))

            # Once the transformation is available, get the transform
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            
            # Log the robot position
            rospy.loginfo("Robot position: %s", trans)
            
            return trans  # Return the position tuple (x, y, z)
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        rate.sleep()

if __name__ == '__main__':
    try:
        position = get_robot_position()
        rospy.loginfo("Final robot position: %s", position)
        
    except rospy.ROSInterruptException:
        pass

