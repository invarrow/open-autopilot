import os

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file = 'audibot.urdf.xacro'
    #xacro_file = "box_bot.xacro"
    package_description = "car_description"

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)
    robot_desc_raw = xacro.process_file(robot_desc_path).toxml()

    # Robot State Publisher

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True,
                'frame_prefix': [LaunchConfiguration('audibot'), '/'],
                'publish_frequency': LaunchConfiguration('tf_freq'),
                     'robot_description': robot_desc_raw}],
        output="screen"
    )

    # create and return launch description object
    return LaunchDescription(
        [
            robot_state_publisher_node,
        ]
    )
