import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    simulation = LaunchConfiguration('simulation', default='true')
    roborscht_description_dir = get_package_share_directory('roborscht_description')

    hand_urdf_file = os.path.join(roborscht_description_dir, 'urdf', 'roborscht_v3.urdf')
    with open(hand_urdf_file, 'r') as infp:
        hand_robot_description = infp.read()

    arm_urdf_file = os.path.join(roborscht_description_dir, 'urdf', 'tm5x-900-nominal.urdf')
    with open(arm_urdf_file, 'r') as infp:
        arm_robot_description = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='hand',
            output='screen',
            parameters=[{
                'use_sim_time': simulation, 
                'robot_description': hand_robot_description
            }],
            arguments=[hand_urdf_file]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='arm',
            output='screen',
            parameters=[{
                'use_sim_time': simulation, 
                'robot_description': arm_robot_description
            }],
            arguments=[arm_urdf_file]
        ),
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     namespace='hand',
        #     output='screen',
        # ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace='arm',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(roborscht_description_dir, 'rviz', 'arm.rviz')]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '-1.571', '0', '0', 'flange', 'base_link']
        ),
    ])