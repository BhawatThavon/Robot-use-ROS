#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Initialize ROS node
rospy.init_node('custom_teleop_keyboard')

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    twist = Twist()

    try:
        while not rospy.is_shutdown():
            # Read keyboard input
            key = get_key()

            # Set linear velocities based on keyboard input
            if key == 'w':
                twist.linear.x = 0.15
                twist.linear.y = 0.0
                twist.angular.z = 0.0
            elif key == 's':
                twist.linear.x = -0.15
                twist.linear.y = 0.0
                twist.angular.z = 0.0
            elif key == 'a':
                twist.linear.x = 0.0
                twist.linear.y = 0.15
                twist.angular.z = 0.0
            elif key == 'd':
                twist.linear.x = 0.0
                twist.linear.y = -0.15
                twist.angular.z = 0.0
            elif key == 'r':
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.15
            elif key == 't':
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = -0.15
            elif key == 'q':
                twist.linear.x = 0.075
                twist.linear.y = 0.075
                twist.angular.z = 0.0
            elif key == 'e':
                twist.linear.x = 0.075
                twist.linear.y = -0.075
                twist.angular.z = 0.0
            elif key == 'z':
                twist.linear.x = -0.075
                twist.linear.y = 0.075
                twist.angular.z = 0.0
            elif key == 'c':
                twist.linear.x = -0.075
                twist.linear.y = -0.075
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0

            # Publish Twist message
            pub.publish(twist)

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Teleoperation terminated by user")
    finally:
        # Stop the robot before exiting and restore terminal settings
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("Terminal settings restored.")


