from ament_index_python.resources import has_resource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description() -> LaunchDescription:
    """
    Generate a launch description for dual camera capture with HD quality and RGB888 format.
    Each camera runs in its own process to avoid the 'Multiple IPAManager objects are not allowed' error.

    Returns
    -------
    LaunchDescription: the launch description
    """
    # Common parameters for both cameras
    format_param = "RGB888"
    width_param = 1280
    height_param = 720
    pkg_share = get_package_share_directory('camera_ros')

    # Define camera IDs - order based on libcamera-hello --list-cameras output
    # cam0_id = "/base/soc/i2c0mux/i2c@1/imx477@1a"  # Camera 0 in the list
    cam1_id = "/base/soc/i2c0mux/i2c@0/imx477@1a"  # Camera 1 in the list

    # cam0_calib_file = os.path.join(pkg_share, 'config', 'cam0_calib.yaml')
    cam1_calib_file = os.path.join(pkg_share, 'config', 'cam1_calib.yaml')


    # Create standalone nodes (not composable)
    # camera0_node = Node(
    #     package='camera_ros',
    #     executable='camera_node',
    #     namespace='cam0',
    #     name='camera0',
    #     output='screen',  # Add this to see output from this node
    #     parameters=[{
    #         "camera": cam0_id,
    #         'camera_info_url': 'file://' + cam0_calib_file,
    #         "width": width_param,
    #         "height": height_param,
    #         "format": format_param,
    #         "trigger_enable": True,
    #         # "AeEnable": False,
    #         # "ExposureTime": 1500
    #     }],
    #     remappings=[
    #         # Make sure topics are properly namespaced
    #         ('~/image_raw', '/cam0/image_raw'),
    #         ('~/camera_info', '/cam0/camera_info')
    #     ],
    # )

    camera1_node = Node(
        package='camera_ros',
        executable='camera_node',
        namespace='cam1',
        name='camera1',
        output='screen',  # Add this to see output from this node
        parameters=[{
            "camera": cam1_id,
            'camera_info_url': 'file://' + cam1_calib_file,
            "width": width_param,
            "height": height_param,
            "format": format_param,
            "trigger_enable": True,
                        "log_level": "debug"

            # "AeEnable": False,
            # "ExposureTime": 1500
        }],
        remappings=[
            # Make sure topics are properly namespaced
            ('~/image_raw', '/cam1/image_raw'),
            ('~/camera_info', '/cam1/camera_info')
        ],
    )

    # Optional image viewer nodes
    viewer_nodes = []
    if has_resource("packages", "image_view"):
        viewer_nodes = [
            Node(
                package='image_view',
                executable='image_view',
                namespace='cam0',
                name='image_view0',
                remappings=[('image', 'image_raw')],
            ),
            Node(
                package='image_view',
                executable='image_view',
                namespace='cam1',
                name='image_view1',
                remappings=[('image', 'image_raw')],
            ),
        ]

    # Helpful logging actions
    log_actions = [
        LogInfo(msg="Starting dual camera nodes with hardware trigger enabled"),
    ]

    # Return the launch description
    return LaunchDescription([
        *log_actions,
        # camera0_node,
        camera1_node,
        *viewer_nodes,
    ])
