import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen


class koch_snow(Node):

    def __init__(self):
        super().__init__('koch_snow')
        self.twist_pub = self.create_publisher(
                            Twist, '/turtle1/cmd_vel', 10)
        self.pose = None
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.cb_pose,
            10)

    # New method for koch_snow
    def cb_pose(self, msg):
        self.pose = msg

    def set_pen(self, r, g, b, width, off):
        pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        while not pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_pen szolgáltatás nem elérhető, várakozás...')
            self.get_logger().info('set_pen szolgáltatás elérhető.')

        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        future = pen_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Toll beállítva.')
        else:
            self.get_logger().error('Hiba történt a toll beállításakor.')

    def go_straight(self, speed, distance):
        while self.pose is None and rclpy.ok():
            self.get_logger().info('Várakozás...')
            rclpy.spin_once(self)

        loop_rate = self.create_rate(100, self.get_clock()) # Hz

        x_start = self.pose.x
        y_start = self.pose.y

        x_end = x_start + distance * math.cos(self.pose.theta)
        y_end = y_start + distance * math.sin(self.pose.theta)

        vel_msg = Twist()
        vel_msg.linear.x = speed if distance > 0 else -speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        self.twist_pub.publish(vel_msg)

        while rclpy.ok():
            # Calculate the current displacement from the start position
            current_displacement = math.sqrt((self.pose.x - x_start)**2 + (self.pose.y - y_start)**2)

            # ellenőrzi, hogy a robot elérte vagy túllépte a távolságot
            if current_displacement >= abs(distance):
                break

            self.twist_pub.publish(vel_msg)
            rclpy.spin_once(self)

        # Stop
        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)
        self.get_logger().info('Megérkezett vagy túllépte a távolságot.')




    def turn(self, omega, angle):
        ANGLE_TOLERANCE = math.radians(0.4)
        omega_rad = math.radians(omega)

        while self.pose is None and rclpy.ok():
            self.get_logger().info('Bátskozás a kezdezi pozícióra...')
            rclpy.spin_once(self)

        angle_rad = math.radians(angle)

        loop_rate = self.create_rate(100, self.get_clock())  # Hz

        theta_start = self.pose.theta

        theta_target = theta_start + angle_rad

        theta_target = (theta_target + math.pi) % (2 * math.pi) - math.pi

        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = omega_rad if angle_rad > 0 else -omega_rad

        self.twist_pub.publish(vel_msg)

        while rclpy.ok():
            current_angle_diff = (self.pose.theta - theta_target + math.pi) % (2 * math.pi) - math.pi

            adjustment_factor = max(abs(current_angle_diff) / angle_rad, 0.1)
            vel_msg.angular.z = omega_rad * adjustment_factor * (1 if angle_rad > 0 else -1)

            if abs(current_angle_diff) < ANGLE_TOLERANCE:
                break

            self.twist_pub.publish(vel_msg)
            rclpy.spin_once(self)

        vel_msg.angular.z = 0.0
        self.twist_pub.publish(vel_msg)
        self.get_logger().info('Elérte a cél pozíciót.')

    def koch_subfunc(self, speed, omega, I, L, P, help):
        print(P)
        self.go_straight(speed, L)
        P=P-60
        self.turn(omega, P)
        print(P)
        self.go_straight(speed, L)
        P=P+180
        self.turn(omega, P)
        print(P)
        self.go_straight(speed, L)
        P=P-180
        self.turn(omega, P)
        print(P)
        self.go_straight(speed, L)
        print(help)
        if help==0:
            self.turn(omega, P)
            self.help=1
        else:
            P=P+180
            self.turn(omega, P)
            self.help=0
    def set_spawnpoint(self, speed, omega, x, y):
        self.set_pen(0, 0, 0, 0, 1)
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.pose is None and rclpy.ok():
            self.get_logger().info('Waiting for pose...')
            rclpy.spin_once(self)

        # Stuff with atan2
        x0 = self.pose.x
        y0 = self.pose.y
        theta_0 = math.degrees(self.pose.theta)

        theta_1 = math.degrees(math.atan2(y-y0, x-x0))
        angle = theta_1 - theta_0
        distance = math.sqrt(((x - x0) * (x - x0)) + (y - y0) * (y - y0))

        # Execute movement
        self.turn(omega, angle)
        self.go_straight(speed, distance)

        self.set_pen(255, 255, 255, 1, 0)
    def draw_koch(self, speed, omega, I, L):
        if I==0:
            self.go_straight(speed, L)
        else:
            self.draw_koch(speed, omega, I-1, L)
            self.turn(omega, 60)
            self.draw_koch(speed, omega, I-1, L)
            self.turn(omega, -120)
            self.draw_koch(speed, omega, I-1, L)
            self.turn(omega, 60)
            self.draw_koch(speed, omega, I-1, L)

def main(args=None):
    rclpy.init(args=args)
    ks = koch_snow()
    ks.set_spawnpoint(5.0,500.0,2,7)
    ks.turn(500, -155)
    for k in range(3):
        ks.draw_koch(1.0,600.0,2,0.2)
        ks.turn(500, -120)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ks.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
