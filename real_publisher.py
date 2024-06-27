#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from std_msgs.msg import Int32
from math import cos, sin, pi
import tf

# Define variables for encoder ticks and wheel parameters
wheel_radius = 0.05  # Radius of the wheels (assuming they are the same)
wheel_distance = 0.485   # Distance between the front and rear wheels
track_width = 0.48      # Distance between the left and right wheels

# Define variables for current pose and orientation
current_x = 0.0
current_y = 0.0
current_theta = 0.0

# Variables to store previous tick counts
prev_front_left_ticks = 0
prev_front_right_ticks = 0
prev_back_left_ticks = 0
prev_back_right_ticks = 0

current_front_left_ticks = 0
current_front_right_ticks = 0
current_back_left_ticks = 0
current_back_right_ticks = 0

def front_left_ticks_callback(msg):
    global current_front_left_ticks
    current_front_left_ticks = msg.data

def front_right_ticks_callback(msg):
    global current_front_right_ticks
    current_front_right_ticks = msg.data

def back_left_ticks_callback(msg):
    global current_back_left_ticks
    current_back_left_ticks = msg.data

def back_right_ticks_callback(msg):
    global current_back_right_ticks
    current_back_right_ticks = msg.data

def publish_odometry(odom_pub, br):
    global current_x, current_y, current_theta
    global prev_front_left_ticks, prev_front_right_ticks, prev_back_left_ticks, prev_back_right_ticks

    # Calculate tick increments
    front_left_ticks_inc = current_front_left_ticks - prev_front_left_ticks
    front_right_ticks_inc = current_front_right_ticks - prev_front_right_ticks
    back_left_ticks_inc = current_back_left_ticks - prev_back_left_ticks
    back_right_ticks_inc = current_back_right_ticks - prev_back_right_ticks

    # Debug print statements for tick increments
    #rospy.loginfo("Tick Increments: FL: %d, FR: %d, BL: %d, BR: %d", front_left_ticks_inc, front_right_ticks_inc, back_left_ticks_inc, back_right_ticks_inc)

    # Update previous tick counts
    prev_front_left_ticks = current_front_left_ticks
    prev_front_right_ticks = current_front_right_ticks
    prev_back_left_ticks = current_back_left_ticks
    prev_back_right_ticks = current_back_right_ticks

    # Calculate wheel velocities
    front_left_velocity = (front_left_ticks_inc / 14900.0) * (2 * pi * wheel_radius)
    front_right_velocity = (front_right_ticks_inc / 14900.0) * (2 * pi * wheel_radius)
    back_left_velocity = (back_left_ticks_inc / 14900.0) * (2 * pi * wheel_radius)
    back_right_velocity = (back_right_ticks_inc / 14900.0) * (2 * pi * wheel_radius)

    # Debug print statements for wheel velocities
    #rospy.loginfo("Wheel Velocities: FL: %f, FR: %f, BL: %f, BR: %f", front_left_velocity, front_right_velocity, back_left_velocity, back_right_velocity)

    # For omnidirectional movement, calculate the robot's velocity in the robot frame
    vx = (front_left_velocity + front_right_velocity + back_left_velocity + back_right_velocity) / 4.0
    vy = (-front_left_velocity + front_right_velocity + back_left_velocity - back_right_velocity) / 4.0
    omega = (-front_left_velocity + front_right_velocity - back_left_velocity + back_right_velocity) / 4.0 #( * (track_width / 2.0))

    # Debug print statements for robot velocities
    #rospy.loginfo("Robot Velocities: vx: %f, vy: %f, omega: %f", vx, vy, omega)

    # Transform robot velocities to the global frame
    delta_x = (vx * cos(current_theta) - vy * sin(current_theta)) /10
    delta_y = (vx * sin(current_theta) + vy * cos(current_theta)) /10
    delta_theta = omega /10

    # Debug print statements for pose updates
    rospy.loginfo("Pose Update: x: %f, y: %f, theta: %f", current_x, current_y, current_theta)
    #rospy.loginfo("Pose Update: delta_x: %f, delta_y: %f, delta_theta: %f", delta_x, delta_y, delta_theta)

    current_x += delta_x
    current_y += delta_y
    current_theta += delta_theta

    # Normalize theta to keep it within -pi to pi
    current_theta = (current_theta + pi) % (2 * pi) - pi

    # Create Odometry message
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"
    odom.pose.pose.position.x = current_x
    odom.pose.pose.position.y = current_y
    odom.pose.pose.position.z = 0.0
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, current_theta)
    odom.pose.pose.orientation = Quaternion(*odom_quat)
    #odom.pose.pose.orientation.z = sin(current_theta / 2.0)
    #odom.pose.pose.orientation.w = cos(current_theta / 2.0)

    odom.twist.twist.linear.x = vx
    odom.twist.twist.linear.y = vy
    odom.twist.twist.angular.z = omega

    # Publish Odometry message
    odom_pub.publish(odom)

    # Create and broadcast TransformStamped message
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
    t.transform.translation.x = current_x
    t.transform.translation.y = current_y
    t.transform.translation.z = 0.0
    t.transform.rotation = Quaternion(*odom_quat)

    # Unpack the TransformStamped fields when calling sendTransform
    br.sendTransform((t.transform.translation.x, t.transform.translation.y, t.transform.translation.z),
                     (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w),
                     t.header.stamp,
                     t.child_frame_id,
                     t.header.frame_id)

    # Publish the transform
    #br.sendTransform((current_x, current_y, 0),
    #                 tf.transformations.quaternion_from_euler(0, 0, current_theta),
    #                 rospy.Time.now(),
    #                 "base_link",
    #                 "odom")

def main():
    rospy.init_node('encoder_to_odometry')

    # Subscribe to wheel tick topics
    rospy.Subscriber('front_left_encoder_ticks', Int32, front_left_ticks_callback)
    rospy.Subscriber('front_right_encoder_ticks', Int32, front_right_ticks_callback)
    rospy.Subscriber('back_left_encoder_ticks', Int32, back_left_ticks_callback)
    rospy.Subscriber('back_right_encoder_ticks', Int32, back_right_ticks_callback)

    # Create Odometry publisher
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)

    # Create a transform broadcaster
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10)  # Publish at 10 Hz

    while not rospy.is_shutdown():
        publish_odometry(odom_pub, br)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


