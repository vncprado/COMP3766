import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('lec10_urdf')

    # Get the path to your plain URDF file (stanford.urdf)
    urdf_file_path = os.path.join(
        pkg_share_dir,
        'urdf',
        'rr_robot.urdf'
    )
    
    # Read the content of the URDF file into a simple Python string
    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Joint State Publisher GUI Node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz Node
    rviz_config_file = PathJoinSubstitution([pkg_share_dir, 'rviz', 'rr_robot.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])