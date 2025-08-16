import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from rclpy.executors import MultiThreadedExecutor
import math

class TurtleBot3WallFollower(Node):
    STATE_FORWARD = 0
    STATE_WAIT = 1
    STATE_TURN_LEFT = 2

    def __init__(self):
        super().__init__('turtlebot3_wall_follower')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.threshold_distance = 0.5  # Increased threshold distance to detect earlier
        self.emergency_stop_distance = 0.15  # Emergency stop if too close
        self.forward_speed = 0.15
        self.turn_speed = 0.4

        self.state = self.STATE_FORWARD
        self.front_distance = float('inf')

        self.start_yaw = None
        self.current_yaw = None
        self.wait_start_time = None

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        num_ranges = len(ranges)
        angle_increment = msg.angle_increment
        desired_angle = math.radians(30)  # 30 degrees left/right

        center_index = num_ranges // 2
        range_indices = int(desired_angle / angle_increment)
        front_indices = range(center_index - range_indices, center_index + range_indices + 1)

        front_ranges = []
        for i in front_indices:
            r = ranges[i]
            if 0.1 < r < float('inf'):
                front_ranges.append(r)

        if front_ranges:
            self.front_distance = min(front_ranges)
        else:
            self.front_distance = msg.range_max

        print(f"[Laser] Front distance (±30°): {self.front_distance:.2f} meters")

        self.update_state_and_motion()

    def odom_callback(self, msg: Odometry):
        orientation_q = msg.pose.pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        self.current_yaw = yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def angular_difference(self, start, current):
        diff = self.normalize_angle(current - start)
        return abs(diff)

    def update_state_and_motion(self):
        twist = Twist()

        if self.state == self.STATE_FORWARD:
            if self.front_distance <= self.threshold_distance:
                print(f"Obstacle detected at {self.front_distance:.2f} meters! Stopping and waiting 1 second before turn.")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.state = self.STATE_WAIT
                self.wait_start_time = self.get_clock().now()
                return
            else:
                twist.linear.x = self.forward_speed
                twist.angular.z = 0.0

        elif self.state == self.STATE_WAIT:
            elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
            if elapsed >= 1.0:
                print("Waited 1 second, starting left turn in place!")
                self.state = self.STATE_TURN_LEFT
                self.start_yaw = self.current_yaw
                twist.linear.x = 0.0
                twist.angular.z = self.turn_speed * -1.0  # Negative for left turn
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        elif self.state == self.STATE_TURN_LEFT:
            if self.start_yaw is None or self.current_yaw is None:
                twist.linear.x = 0.0
                twist.angular.z = self.turn_speed * -1.0
            else:
                turned = self.angular_difference(self.start_yaw, self.current_yaw)
                if turned >= (math.pi / 2) - 0.05:
                    print("Turned 90 degrees left, stopping turn and scanning ahead.")
                    self.state = self.STATE_FORWARD
                    self.start_yaw = None
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = self.turn_speed * -1.0

        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            print("Unknown state: stopping robot.")

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3WallFollower()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

