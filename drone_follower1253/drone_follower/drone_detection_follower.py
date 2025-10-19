#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json, time

class DroneDetectionFollower(Node):
    def __init__(self):
        super().__init__('drone_detection_follower')
        self.cmd_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.detection_sub = self.create_subscription(String, '/yolov3/detections', self.detection_callback, 10)
        self.last_detection_time = 0.0
        self.timeout = 2.0
        self.kp_x = 0.002
        self.kp_y = 0.002
        self.center_x = 640 / 2
        self.center_y = 480 / 2
        self.timer = self.create_timer(0.1, self.control_loop)
        self.last_detection = {}
        self.get_logger().info('DroneDetectionFollower node started')

    def detection_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, list):
                data = data[0]
            self.last_detection = data
            self.last_detection_time = time.time()
        except Exception as e:
            self.get_logger().warn(f'Failed to parse detection: {e}')

    def control_loop(self):
        now = time.time()
        cmd = Twist()
        if now - self.last_detection_time < self.timeout:
            x = self.last_detection.get('x', self.center_x)
            y = self.last_detection.get('y', self.center_y)
            dx = x - self.center_x
            dy = y - self.center_y
            cmd.linear.y = -self.kp_x * dx
            cmd.linear.x = -self.kp_y * dy
            self.get_logger().info(f'Tracking target dx={dx:.1f}, dy={dy:.1f}')
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DroneDetectionFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
