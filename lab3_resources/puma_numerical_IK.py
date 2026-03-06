"""
puma_numerical_IK.py
Students should edit this scritp's function "puma_numerical_IK_calc" with their solution to Numerical IK for the puma.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose


class GoalPoseSubscriber(Node):

    def __init__(self):
        super().__init__('puma_numerical_IK')
        self.subscription = self.create_subscription(Pose, 'goal_pose', self.goal_pose_callback, 10)
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 
            'joint4', 'joint5', 'joint6'
        ]

    def goal_pose_callback(self, pose_msg):
        self.get_logger().info(f'I heard orientation: {pose_msg.orientation}')
        self.get_logger().info(f'I heard orientation: {pose_msg.position}')

        puma_joints_msg = JointState()
        puma_joints_msg.header.stamp = self.get_clock().now().to_msg()
        puma_joints_msg.name = self.joint_names
        puma_joints_msg.position = self.puma_numerical_IK_calc(pose_msg)

        self.get_logger().info(f'Publishing numerical PUMA joints: {puma_joints_msg.position}')
        self.publisher_.publish(puma_joints_msg)

    def puma_numerical_IK_calc(self, pose_msg):
        # Edit this function to get the joint positions based on "pose_msg"
        # Convert your pose to the required position and orientation

        return [0.0] * len(self.joint_names) # placeholder


def main(args=None):
    rclpy.init(args=args)

    goal_pose_subscriber = GoalPoseSubscriber()
    rclpy.spin(goal_pose_subscriber)

    goal_pose_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
