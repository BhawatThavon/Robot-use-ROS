#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion
from tf.transformations import euler_from_quaternion
import math

def send_goal(x, y, yaw):
    # Initialize a ROS node
    rospy.init_node('send_navigation_goal')

    # Create an action client for move_base
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait for the action server to come up
    client.wait_for_server()

    # Create a goal object
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  # Reference frame for the goal
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the goal position
    goal.target_pose.pose.position = Point(x, y, 0)

    # Set the goal orientation (quaternion)
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = 0.0
    q.w = 1.0
    goal.target_pose.pose.orientation = q

    # Send the goal to the action server
    rospy.loginfo("Sending goal: (%f, %f)", x, y)
    client.send_goal(goal)

    # Wait for the result
    client.wait_for_result()

    # Check if the goal succeeded
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached!")

        # Log robot's final position
        final_position = get_robot_position()
        rospy.loginfo("Final robot position: %s", final_position)

        # Calculate and log distance error
        target_pos = (x, y)
        distance_error = calculate_error(target_pos, final_position)
        rospy.loginfo("Distance error: %.2f meters", distance_error)

        # Calculate and log yaw error
        target_yaw = yaw
        actual_yaw = get_robot_yaw()
        yaw_error = calculate_yaw_error(target_yaw, actual_yaw)
        rospy.loginfo("Yaw error: %.2f radians", yaw_error)
    else:
        rospy.logwarn("Failed to reach the goal!")

def get_robot_position():
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            return trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

def get_robot_yaw():
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            euler = euler_from_quaternion(rot)
            return euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

def calculate_error(target, actual):
    dx = target[0] - actual[0]
    dy = target[1] - actual[1]
    distance_error = math.sqrt(dx**2 + dy**2)
    return distance_error

def calculate_yaw_error(target_yaw, actual_yaw):
    error = abs(target_yaw - actual_yaw)
    return error

if __name__ == '__main__':
    try:
        # Replace with your desired goal coordinates
        target_x = 9.0
        target_y = 5.0
        target_yaw = 0.0

        send_goal(target_x, target_y, target_yaw)
    except rospy.ROSInterruptException:
        pass

