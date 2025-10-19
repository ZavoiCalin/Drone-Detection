#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import json
import math
import time

class DroneFollower(Node):
    def __init__(self):
        super().__init__('drone_follower')

        # Parameters (can be changed by modifying the node code or via ROS2 params later)
        self.declare_parameter('follow_distance', 3.0)
        self.declare_parameter('linear_kp', 0.5)
        self.declare_parameter('angular_kp', 1.0)
        self.declare_parameter('detection_timeout', 2.0)
        self.declare_parameter('detection_confidence', 0.7)

        self.follow_distance = self.get_parameter('follow_distance').value
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.detection_confidence = self.get_parameter('detection_confidence').value

        # Subscribers
        self.create_subscription(String, '/yolov3/detections', self.detection_callback, 10)
        self.create_subscription(PoseStamped, '/drone_self/pose', self.self_pose_callback, 10)
        self.create_subscription(PoseStamped, '/drone_target/pose', self.target_pose_callback, 10)

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/drone_self/cmd_vel', 10)

        # Internal state
        self.self_pose = None
        self.target_pose = None
        self.last_detection_time = 0.0
        self.target_visible = False

        # Control loop timer (10 Hz)
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Drone follower node started.")

    def detection_callback(self, msg):
        """Parse YOLO JSON and check if a 'drone' object was detected."""
        try:
            detections = json.loads(msg.data)
            for det in detections:
                if det.get('object') == 'drone' and det.get('confidence', 0) >= self.detection_confidence:
                    self.target_visible = True
                    self.last_detection_time = time.time()
                    # Optionally store bbox if needed: det.get('bbox')
                    self.get_logger().info('Target drone detected (confidence: {:.2f})'.format(det.get('confidence', 0)))
                    return
        except Exception as e:
            self.get_logger().warn(f'Bad detection JSON: {e}')

    def self_pose_callback(self, msg):
        self.self_pose = msg

    def target_pose_callback(self, msg):
        self.target_pose = msg

    def control_loop(self):
        """Called at 10 Hz. If target visible, compute velocity command toward it."""
        if not self.self_pose or not self.target_pose:
            # Not enough info yet
            return

        # If detection too old, stop
        if not self.target_visible or (time.time() - self.last_detection_time > self.detection_timeout):
            if self.target_visible:
                self.get_logger().info('Detection timed out — stopping/hovering.')
            self.stop_drone()
            self.target_visible = False
            return

        # Compute vector to target (2D planar)
        dx = self.target_pose.pose.position.x - self.self_pose.pose.position.x
        dy = self.target_pose.pose.position.y - self.self_pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        # Build Twist command
        cmd = Twist()

        # Proportional control to maintain follow_distance
        if distance > self.follow_distance + 0.1:
            # move forward proportional to the error
            speed = self.linear_kp * (distance - self.follow_distance)
            # clamp speed to a reasonable max (e.g., 2.0 m/s)
            max_speed = 2.0
            cmd.linear.x = max(-max_speed, min(max_speed, speed))
            # angle to target
            angle = math.atan2(dy, dx)
            # For simplicity we publish angular.z proportional to angle (assumes controller or robot interprets it)
            cmd.angular.z = self.angular_kp * angle
        else:
            # Close enough: stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def stop_drone(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DroneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
