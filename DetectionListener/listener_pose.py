import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import json
import math

class GazeboDetectionNode(Node):
    def __init__(self):
        super().__init__('gazebo_detection_node')

        # YOLO detections (JSON string)
        self.create_subscription(String, '/yolov3/detections', self.detection_callback, 10)

        # Drone poses (world coordinates)
        self.drone_positions = {}  # drone_id: PoseStamped
        drone_topics = ['/drone1/pose', '/drone2/pose', '/drone3/pose']
        for topic in drone_topics:
            drone_id = topic.split('/')[1]
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, drone_id=drone_id: self.pose_callback(msg, drone_id),
                10
            )

        # Marker publisher
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        self.latest_detections = []

    def pose_callback(self, msg, drone_id):
        self.drone_positions[drone_id] = msg

    def detection_callback(self, msg):
        # YOLO detections assumed as JSON list:
        # [{"object": "drone", "confidence": 0.95, "bbox": [x_min, y_min, x_max, y_max]}]
        detections = json.loads(msg.data)
        self.latest_detections = detections
        self.publish_markers()

    def publish_markers(self):
        for detection in self.latest_detections:
            # Optionally, find nearest drone (simulation)
            nearest_drone_id, min_dist = None, float('inf')
            for drone_id, pose in self.drone_positions.items():
                dx = pose.pose.position.x
                dy = pose.pose.position.y
                dz = pose.pose.position.z
                dist = math.sqrt(dx**2 + dy**2 + dz**2)  # distance from origin for example
                if dist < min_dist:
                    min_dist = dist
                    nearest_drone_id = drone_id

            # Publish marker at the drone position
            if nearest_drone_id:
                pose = self.drone_positions[nearest_drone_id]
                marker = Marker()
                marker.header.frame_id = 'world'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'detection'
                marker.id = hash(detection["object"]) % 1000
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose = pose.pose
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.5
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                self.marker_pub.publish(marker)
                self.get_logger().info(f"Published marker for {detection['object']} at {pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z}")

def main():
    rclpy.init()
    node = GazeboDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
