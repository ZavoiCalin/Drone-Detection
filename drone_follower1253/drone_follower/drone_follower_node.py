#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist

class DroneFollower(Node):
    def __init__(self):
        super().__init__('drone_follower_node')
        self.create_subscription(PoseStamped, '/drone_target/pose', self.target_callback, 10)
        self.create_subscription(PoseStamped, '/drone_self/pose', self.self_callback, 10)
        self.pub = self.create_publisher(Twist, '/drone_self/cmd_vel', 10)
        self.target = None
        self.self_pose = None
        self.timer = self.create_timer(0.1, self.follow)

    def target_callback(self, msg):
        self.target = msg

    def self_callback(self, msg):
        self.self_pose = msg

    def follow(self):
        if self.target and self.self_pose:
            cmd = Twist()
            cmd.linear.x = 0.5 * (self.target.pose.position.x - self.self_pose.pose.position.x)
            cmd.linear.y = 0.5 * (self.target.pose.position.y - self.self_pose.pose.position.y)
            self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DroneFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
