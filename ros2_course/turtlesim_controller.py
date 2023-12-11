import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtlesimController(Node):

    def __init__(self):
        super().__init__('turtlesim_controller')
        self.twist_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def go_straight(self, speed, distance):
        vel_msg = Twist()

        # Set linear velocity based on direction
        if distance > 0:
            vel_msg.linear.x = speed
        else:
            vel_msg.linear.x = -speed

        # Set other components of Twist message
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        # Set loop rate
        loop_rate = self.create_rate(100, self.get_clock())  # Hz

        # Calculate time
        T = abs(distance) / abs(speed)

        # Publish first message and note the time when to stop
        self.twist_pub.publish(vel_msg)
        self.get_logger().info('Turtle started.')
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        # Publish message while the calculated time is not up
        while (self.get_clock().now() < when) and rclpy.ok():
            self.twist_pub.publish(vel_msg)
            self.get_logger().info('On its way...')
            rclpy.spin_once(self)  # loop rates

        # Turtle arrived, set velocity to 0
        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)
        self.get_logger().info('Arrived at the destination.')

def main(args=None):
    rclpy.init(args=args)
    tc = TurtlesimController()

    # Example usage: Move forward 2 units with a speed of 1
    tc.go_straight(speed=1.0, distance=2.0)

    # Destroy the node explicitly
    # (optional - otherwise, it will be done automatically
    # when the garbage collector destroys the node object)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
