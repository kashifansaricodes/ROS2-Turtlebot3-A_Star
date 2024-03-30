import rclpy
from geometry_msgs.msg import Twist

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('turtlebot_controller')

    # Create a ROS 2 publisher for velocity commands
    vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    # Create a Twist message object
    twist_msg = Twist()

    # Set linear and angular velocity values
    twist_msg.linear.x = 0.2  # Example linear velocity in m/s
    twist_msg.angular.z = 0.5  # Example angular velocity in rad/s

    # Publish the Twist message
    vel_pub.publish(twist_msg)

    # Spin the node to execute callbacks
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
