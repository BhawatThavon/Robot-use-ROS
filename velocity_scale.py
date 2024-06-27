#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

MAX_COMBINED_VEL = 0.15  # Maximum allowed combined velocity

def scale_velocity(cmd_vel):
    # Calculate individual motor speeds based on mecanum wheel formulas
    v_fl = cmd_vel.linear.x - cmd_vel.linear.y - cmd_vel.angular.z
    v_fr = cmd_vel.linear.x + cmd_vel.linear.y + cmd_vel.angular.z
    v_bl = cmd_vel.linear.x + cmd_vel.linear.y - cmd_vel.angular.z
    v_br = cmd_vel.linear.x - cmd_vel.linear.y + cmd_vel.angular.z

    # Calculate the maximum absolute motor speed
    max_motor_speed = max(abs(v_fl), abs(v_fr), abs(v_bl), abs(v_br))

    # Scale the velocities if the maximum motor speed exceeds the limit
    if max_motor_speed > MAX_COMBINED_VEL:
        scale_factor = MAX_COMBINED_VEL / max_motor_speed
        cmd_vel.linear.x *= scale_factor
        cmd_vel.linear.y *= scale_factor
        cmd_vel.angular.z *= scale_factor

    return cmd_vel

def cmd_vel_callback(msg):
    scaled_vel = scale_velocity(msg)
    pub.publish(scaled_vel)

if __name__ == '__main__':
    rospy.init_node('velocity_scaling_node')
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    pub = rospy.Publisher('cmd_vel_scaled', Twist, queue_size=10)
    rospy.spin()

