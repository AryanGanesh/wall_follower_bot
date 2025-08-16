import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor

class TurtleBot3WallFollower(Node):
    STATE_FORWARD = 0
    STATE_TURN_LEFT = 1

    def __init__(self):
        super().__init__('turtlebot3_wall_follower')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        self.target_distance = 0.3
        self.distance_tolerance = 0.05
        self.forward_speed = 0.15
        self.turn_speed = 0.4
        self.turn_duration = 2.0  # seconds, adjust if needed

        self.state = self.STATE_FORWARD
        self.turn_start_time = None

        self.front_distance = float('inf')

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        num_ranges = len(ranges)

        front_indices = [i for i in range(num_ranges//2 - 10, num_ranges//2 + 10)]
        front_ranges = [ranges[i] for i in front_indices if 0.1 < ranges[i] < float('inf')]

        self.front_distance = min(front_ranges) if front_ranges else float('inf')

        self.do_behavior()

    def do_behavior(self):
        twist = Twist()

        if self.state == self.STATE_FORWARD:
            if self.front_distance < self.target_distance:
                self.get_logger().info(f"Obstacle ahead at {self.front_distance:.2f}m, starting left turn")
                self.state = self.STATE_TURN_LEFT
                self.turn_start_time = self.get_clock().now()
                twist.linear.x = 0.0
                twist.angular.z = self.turn_speed
            else:
                twist.linear.x = self.forward_speed
                twist.angular.z = 0.0

        elif self.state == self.STATE_TURN_LEFT:
            elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9
            if elapsed >= self.turn_duration:
                self.get_logger().info(f"Finished left turn after {elapsed:.2f}s, resuming forward")
                self.state = self.STATE_FORWARD
                twist.linear.x = self.forward_speed
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = self.turn_speed

        else:
            self.get_logger().warn(f"Unknown state {self.state}, stopping")
            twist.linear.x = 0.0
            twist.angular.z = 0.0

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

