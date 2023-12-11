import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Color
from turtlesim.srv import SetPen

class TurtleControl(Node):
    def __init__(self):
        super().__init__('turtle_control')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.color_publisher = self.create_publisher(Color, 'turtle1/color_sensor', 10)

        self.timer = self.create_timer(1.0, self.update)  # Timer to change color every 1 second

        self.linear_velocity = 1.0  # Set your desired linear velocity
        self.angular_velocity = 1.0  # Set your desired angular velocity

        self.pen_colors = [
            (255, 0, 0),  # Red
            (0, 255, 0),  # Green
            (0, 0, 255),  # Blue
        ]
        self.color_index = 0

    def update(self):
        # Move the turtle forward
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        twist_msg.angular.z = self.angular_velocity
        self.publisher.publish(twist_msg)

        # Change pen color periodically
        color_msg = Color()
        color_msg.r, color_msg.g, color_msg.b = self.pen_colors[self.color_index]
        SetPen(0,250,250)
        self.color_publisher.publish(color_msg)

        # Update color index for the next iteration
        self.color_index = (self.color_index + 1) % len(self.pen_colors)


def main(args=None):
    rclpy.init(args=args)

    turtle_control = TurtleControl()

    try:
        rclpy.spin(turtle_control)
    except KeyboardInterrupt:
        pass

    turtle_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
