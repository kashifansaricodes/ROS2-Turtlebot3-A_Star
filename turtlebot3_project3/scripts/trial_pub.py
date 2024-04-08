#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('turtlebot_controller')
    vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    twist_msg = Twist()
    twist_msg.linear.x = 0.2  # linear velocity in m/s
    twist_msg.angular.z = 0.5  # angular velocity in rad/s

    rate = node.create_rate(10)  # Create a rate object for publishing at 10 Hz

    while rclpy.ok():
        vel_pub.publish(twist_msg)
        print("Published velocity command", twist_msg)
        rate.sleep()  # Sleep to maintain the desired rate

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    