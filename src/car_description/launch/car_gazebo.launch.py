import os
import xacro
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'car_description'
    xacro_file_name= 'audibot.urdf.xacro'
    #in xml
    xacro_file_path = os.path.join(get_package_share_directory(package_name),'urdf', xacro_file_name)
    robot_description_raw = xacro.process_file(xacro_file_path).toxml()

    # Declare launch arguments for model position and orientation

    # Node to publish the robot state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time':True,
                     }],
        output='screen',
    )

    # Node to publish joint states take world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': os.path.join(get_package_share_directory(package_name), 'worlds', 'cararena.world')}.items()

    )

    spawn_entity = Node(
        package='gazebo_ros',
        name='car_spawner',
        executable='spawn_entity.py',
        arguments=['-entity', 'car', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '50.0', '-Y', '0.0'],
        output='screen'
    )

    # Launch description
    return LaunchDescription([

        robot_state_publisher_node,
        gazebo,
        spawn_entity,

    ])

if __name__ == '__main__':
    generate_launch_description()

