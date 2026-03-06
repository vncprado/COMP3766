"""
goal_pose_publisher.py
This node oublishes a desired pose for the PUMA robot
"""

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

# Global constants for the goal pose
GOAL_POSITION = np.array([0.4, 0.1, 0.0])
GOAL_ROTATION_MATRIX = np.array([
    [ 1.0, 0.0,  0.0],  
    [ 0.0, 1.0,  0.0],
    [ 0.0, 0.0,  1.0]
])

class PosePublisher(Node):

    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(Pose, 'goal_pose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # Convert rotation matrix to quaternion
        rotation_matrix_4x4 = np.eye(4)
        rotation_matrix_4x4[:3, :3] = GOAL_ROTATION_MATRIX
        quaternion = R.from_matrix(GOAL_ROTATION_MATRIX).as_quat()

        # Define the goal pose
        goal_pose_msg = Pose()
        goal_pose_msg.position.x = GOAL_POSITION[0]
        goal_pose_msg.position.y = GOAL_POSITION[1]
        goal_pose_msg.position.z = GOAL_POSITION[2]

        goal_pose_msg.orientation.x = quaternion[0]
        goal_pose_msg.orientation.y = quaternion[1]
        goal_pose_msg.orientation.z = quaternion[2]
        goal_pose_msg.orientation.w = quaternion[3]

        self.publisher_.publish(goal_pose_msg)

        self.get_logger().info("Publishing goal pose...")
        self.get_logger().info(f"Goal Position: {GOAL_POSITION}")
        self.get_logger().info(f"Goal Rotation Matrix:\n{GOAL_ROTATION_MATRIX}")
        self.get_logger().info(f"Converted Quaternion: {quaternion}")

        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    pose_publisher = PosePublisher()

    rclpy.spin(pose_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
