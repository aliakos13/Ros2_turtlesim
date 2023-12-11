import cv2
import numpy as np
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen


class ImageDrawer(Node):
    def __init__(self):
        super().__init__("image_drawer")
        self.twist_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose = None
        self.subscription = self.create_subscription(
            Pose, "/turtle1/pose", self.cb_pose, 10
        )

    def cb_pose(self, msg):
        self.pose = msg

    def set_pen(self, r, g, b, width, off):
        pen_client = self.create_client(SetPen, "/turtle1/set_pen")
        while not pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("set_pen service not available, waiting...")
            self.get_logger().info("set_pen service available.")

        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        future = pen_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Pen set.'R: {r}, G: {g}, B: {b}'")
        else:
            self.get_logger().error("Error setting pen.")

    def draw_image(self, image_path, scale=1.0):
        self.get_logger().info("Getting to starting point")

        # Load the image
        image = cv2.cvtColor(cv2.imread(image_path), cv2.COLOR_BGR2RGB)

        # Check if the image is loaded successfully
        if image is None:
            self.get_logger().error(
                "Failed to load the image. Check the image path and format."
            )
            return
        self.get_logger().info("Image loaded")
        height, width = image.shape[:2]
        self.set_pen(0, 255, 0, 1, 0)

        for y in range(5, 10):
            for x in range(1, 10):
                self.go_to_point(1,1)
                #self.set_pen(0, 0, 0, 0, 1)
                #self.go_to_point(2 + (x * 0.05), 7 - (y * 0.05))
                #color = image[x, y]
                #self.set_pen(int(color[0]), int(color[1]), int(color[2]), 5, 0)
                #self.go_to_point(2.05 + (x * 0.05), 6.95 - (y * 0.05))



    def go_to_start_point(self, x, y):
        self.set_pen(0, 0, 0, 0, 1)
        self.set_spawnpoint(0.0, 0.0, x, y)

    def go_to_point(self, x, y):
        self.get_logger().info(f"Go to point: {x}, {y}")
        distance = math.sqrt((x - self.pose.x) ** 2 + (y - self.pose.y) ** 2)
        angle = math.atan2(y - self.pose.y, x - self.pose.x) - math.radians(
            self.pose.theta
        )
        self.turn(100, math.degrees(angle))
        self.go_straight(1.0, distance)
        self.get_logger().info(f"Reached: {x}, {y}")

    def set_spawnpoint(self, speed, omega, x, y):
        self.set_pen(0, 0, 0, 0, 1)
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock())  # Hz
        while self.pose is None and rclpy.ok():
            self.get_logger().info("Waiting for pose...")
            rclpy.spin_once(self)

        # Stuff with atan2
        x0 = self.pose.x
        y0 = self.pose.y
        theta_0 = math.degrees(self.pose.theta)

        theta_1 = math.degrees(math.atan2(y - y0, x - x0))
        angle = theta_1 - theta_0
        distance = math.sqrt(((x - x0) * (x - x0)) + (y - y0) * (y - y0))

        # Execute movement
        self.turn(omega, angle)
        self.go_straight(speed, distance)

        self.set_pen(0, 255, 0, 1, 0)

    def turn(self, omega, angle):
        # Implement straght motion here
        # Create and publish msg
        vel_msg = Twist()

        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        if angle > 0:
            vel_msg.angular.z = math.radians(omega)
        else:
            vel_msg.angular.z = -math.radians(omega)


        # Set loop rate
        loop_rate = self.create_rate(100, self.get_clock()) # Hz

        # Calculate time
        T = abs(angle / omega)

        # Publish first msg and note time when to stop
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Turtle started.')
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        # Publish msg while the calculated time is up
        while (self.get_clock().now() <= when) and rclpy.ok():
            self.twist_pub.publish(vel_msg)
            #self.get_logger().info('On its way...')
            rclpy.spin_once(self)   # loop rate

        # turtle arrived, set velocity to 0
        vel_msg.angular.z = 0.0
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Arrived to destination.')


    def go_straight(self, speed, distance):
        while self.pose is None and rclpy.ok():
            self.get_logger().info("Waiting...")
            rclpy.spin_once(self)

        loop_rate = self.create_rate(100, self.get_clock())  # Hz

        x_start = self.pose.x
        y_start = self.pose.y

        x_end = x_start + distance * math.cos(self.pose.theta)
        y_end = y_start + distance * math.sin(self.pose.theta)

        vel_msg = Twist()
        vel_msg.linear.x = float(speed) if distance > 0 else -float(speed)
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        self.twist_pub.publish(vel_msg)

        while rclpy.ok():
            current_displacement = math.sqrt(
                (self.pose.x - x_start) ** 2 + (self.pose.y - y_start) ** 2
            )

            if current_displacement >= abs(distance):
                break

            self.twist_pub.publish(vel_msg)
            rclpy.spin_once(self)

        # Stop
        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    drawer = ImageDrawer()
    drawer.draw_image("/home/ros_user/Downloads/LEGO60.png", scale=1)
    drawer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
