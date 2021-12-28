import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os

def generate_launch_description():
    # declare the launch args to read for this file
    config = os.path.join(
        get_package_share_directory('riptide_mapping2'),
        'config',
        'config.yaml'
        )

    return launch.LaunchDescription([
        # create the nodes    
        launch_ros.actions.Node(
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