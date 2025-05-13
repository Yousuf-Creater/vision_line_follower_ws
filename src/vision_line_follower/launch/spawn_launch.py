from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('vision_line_follower')

    # URDF processing
    xacro_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # World file path
    world_file = os.path.join(pkg_path, 'worlds', 'my_custom_world.world')

    return LaunchDescription([
        # Launch Gazebo with your custom world
        ExecuteProcess(
            cmd=['gazebo', world_file, '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

         # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        
        # Publish robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        # Spawn the robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-topic', 'robot_description'],
            output='screen'
        ),

        # Launch your line follower node
        Node(
            package='vision_line_follower',
            executable='Line_follower',
            name='Line_follower',
            output='screen'
        )
    ])
