"""
puma_trajectory.py
Instructions:
    * Edit this scritp to implement the trajectory generation for the PUMA robot.
    * Use the CartesianTrajectory or ScrewTrajectory from modern_robotics libarry to calculate a trajectory
    * Calculate the IK for each pose in the trajectory using Lab 3 code
    * Publish joint states at the correct rate
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
from scipy.spatial.transform import Rotation as R
import modern_robotics as mr

import time

np.set_printoptions(suppress=True, precision=4, linewidth=150)

class GoalPoseSubscriber(Node):

    def __init__(self):
        super().__init__('puma_trajectory_subscriber')

        self.is_initialized = False

        self.subscription = self.create_subscription(Pose, 'goal_pose', self.goal_pose_callback, 10)
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 
            'joint4', 'joint5', 'joint6'
        ]
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publish zero to PUMA joints
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [0.0] * len(self.joint_names)
        
        self.publisher_.publish(msg)
        self.get_logger().info('Sent initial zero positions to TF.')
        
        self.is_initialized = True
        
        
    def PoseToHTrans(self, pose):
        """
        Converts a Pose to numpy array representing a 4x4 homogeneous transformation matrix
        :param pose: geometry_msgs.msg.Pose
        :return: A homogeneous transformation matrix representation of the input
        """
        
        t = [pose.position.x, pose.position.y, pose.position.z]
        q = [pose.orientation.x,
             pose.orientation.y, 
             pose.orientation.z,
             pose.orientation.w]
        
        T = np.eye(4)
        T[:3, :3] = R.from_quat(q).as_matrix()
        T[:3, 3] = t
        
        return T
    

    def TransStampedToHTrans(self, ts):
        """
        Converts a TransformStamped message to a 4x4 homogeneous matrix.
        :param ts: geometry_msgs.msg.TransformStamped
        :return: A homogeneous transformation matrix representation of the input
        """
        
        t = [ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z]
        q = [ts.transform.rotation.x, 
             ts.transform.rotation.y, 
             ts.transform.rotation.z, 
             ts.transform.rotation.w]
        
        T = np.eye(4)
        T[:3, :3] = R.from_quat(q).as_matrix()
        T[:3, 3] = t
        
        return T


    def GetCurrentTsb(self):
        """
        Lookup the current transformation Tsb
        :return: StampedTransform from {s} to {b} if available, None otherwise
        """

        try:
            now = rclpy.time.Time()
            Tsb = self.tf_buffer.lookup_transform('{s}','{b}', now)
            self.get_logger().info(f'Found transform "{"b"}" to "{"s"}"')
            
            return Tsb

        except TransformException as e:
            self.get_logger().info(f'Could not find transform "{"b"}" to "{"s"}": {e}')

            return None
    

    def goal_pose_callback(self, goal_pose_msg):
        """
        This method receives goal_pose_msg from the lab3's goal_pose_publisher as the final pose Xend for your trajectory
        Get the current end-edffectror StampedTransform Tsb, i.e., the initial transformation Xstart
        TODO: Use the mr.CartesianTrajectory or mr.ScrewTrajectory to get a trajectory
        For each HTrans in the trajectory, calculate the IK
        Publish the list of joint lists respecting the time definition Tf and N
        """
        
        if not self.is_initialized:
            self.get_logger().warn('Received goal_pose but TF is not initialized yet. Skipping...')
            return
        
        current_pose = self.GetCurrentTsb()
        if current_pose == None:
            self.get_logger().info("No transformation found, no trajectory possible.")
            rclpy.shutdown()
            return None
        
        Xstart = self.TransStampedToHTrans(current_pose) # The initial end-effector configuration
        Xend = self.PoseToHTrans(goal_pose_msg) # The final end-effector configuration

        self.get_logger().info(f'Xstart:\n{Xstart}')
        self.get_logger().info(f'Xend:\n{Xend}')
        
        Tf = 5 # Total time of the motion in seconds from rest to rest
        N = 10 # The number of points N > 1 (Start and stop) in the discrete representation of the trajectory
        method = 3 # cubic (third-order polynomial) time scaling
        
        delay = Tf / (N - 1)
        
        # Use mr.ScrewTrajectory or mr.CartesianTrajectory to get a list of poses
        # Loop over the trajectory calculating the IK for each pose
        # Publish the joints found in the IK solver to joint_states, look the __init__ function for a sample
        # call time.sleep(delay) after each joint published


        # YOUR CODE HERE


        self.get_logger().info(f'Trajectory complete!')
        rclpy.shutdown()
        

    def puma_numerical_IK_calc(self, X):
        """
        TODO: Edit this function to get the IK joint positions for X (pose)
        Use Lab 3 numerical inverse kinematics for the PUMA robot
        """
        
        # YOUR CODE HERE

        return [0.0] * len(self.joint_names) # placeholder


def main(args=None):
    rclpy.init(args=args)

    goal_pose_subscriber = GoalPoseSubscriber()
    rclpy.spin(goal_pose_subscriber)

    goal_pose_subscriber.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()


            
