import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # declare the launch args to read for this file
    config = os.path.join(
        get_package_share_directory('riptide_mapping2'),
        'config',
        'config.yaml'
        )

    return launch.LaunchDescription([

        DeclareLaunchArgument(
            "log_level", 
            default_value="INFO",
            description="log level to use",
        ),

        PushRosNamespace('tempest'),

        # create the nodes    
        Node(
            package='riptide_mapping2',
            executable='mapping',
            name='riptide_mapping2',
            respawn=True,
            output='screen',
            
            # use the parameters on the node
            parameters = [
                config
            ]
        )
    ])