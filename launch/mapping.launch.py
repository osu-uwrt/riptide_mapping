import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import math
import os
from launch.actions import DeclareLaunchArgument

cfg_36h11 = {
    "image_transport": "raw",
    "family": "36h11",
    "size": 0.12065,
    "max_hamming": 0,
    "z_up": True
}

def generate_launch_description():
    # declare the launch args to read for this file
    config = os.path.join(
        get_package_share_directory('riptide_mapping2'),
        'config',
        'config.yaml'
        )

    tag_node = ComposableNode(
        name='apriltag_36h11',
        namespace='apriltag',
        package='apriltag_ros', plugin='AprilTagNode',
        remappings=[
            # This maps the 'raw' images for simplicity of demonstration.
            # In practice, this will have to be the rectified 'rect' images.
            ("/apriltag/image_rect", "/zed2i/zed_node/right_raw/image_raw_color"),
            ("/apriltag/camera_info", "/zed2i/zed_node/right_raw/camera_info"),
        ],
        parameters=[cfg_36h11],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[tag_node],
        output='screen'
    )

    surface_frame_node = Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='surface_frame_node',
                    arguments=["0", "1", "0", f"{math.radians(-90)}", f"{math.radians(-90)}", "0", "tag36h11:0", "estimated_origin_frame"]
                )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "log_level", 
            default_value="INFO",
            description="log level to use",
        ),

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
        ),
        
        container,

        surface_frame_node
    ])