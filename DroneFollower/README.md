# drone_follower

A simple ROS 2 Humble (ament_python) package that implements a **drone follower** node for Gazebo simulations.

- Subscribes:
  - `/yolov3/detections` (std_msgs/String) — JSON list of detections.
  - `/drone_self/pose` (geometry_msgs/PoseStamped) — follower drone pose.
  - `/drone_target/pose` (geometry_msgs/PoseStamped) — target drone pose.

- Publishes:
  - `/drone_self/cmd_vel` (geometry_msgs/Twist) — velocity commands to move the follower.

Install: place this package folder inside your workspace `src/`, then:
```bash
cd ~/ros2_ws
colcon build
. install/setup.bash
ros2 run drone_follower drone_follower_node
```

Notes:
- This is a simple proportional controller for demo/testing in Gazebo.
- Topics names can be adjusted as needed.
