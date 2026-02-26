import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

package_share = FindPackageShare('rovtio').find('rovtio')
config_file = os.path.join(package_share, 'cfg/rovtio', 'rovtio.info')

def generate_launch_description():

    # -------------------------------------------------------------------------
    # Declare launch arguments (override from CLI with arg:=value)
    # -------------------------------------------------------------------------

    declared_args = [

        # --- Frame names ---
        DeclareLaunchArgument('map_frame',    default_value='/map'),
        DeclareLaunchArgument('world_frame',  default_value='/world'),
        DeclareLaunchArgument('camera_frame', default_value='/camera'),
        DeclareLaunchArgument('imu_frame',    default_value='/imu'),

        # --- Topics ---
        DeclareLaunchArgument('imu_topic',  default_value='/imu0'),
        DeclareLaunchArgument('cam0_topic', default_value='/cam0/image_raw'),
        DeclareLaunchArgument('cam1_topic', default_value='/cam1/image_raw'),

        # --- Camera time offsets (seconds, t_imu = t_cam + offset) ---
        DeclareLaunchArgument('cam0_offset', default_value='0.0'),
        DeclareLaunchArgument('cam1_offset', default_value='0.0'),

        # --- Image resizing ---
        DeclareLaunchArgument('resize_image',        default_value='false'),
        DeclareLaunchArgument('resize_image_width',  default_value='320'),
        DeclareLaunchArgument('resize_image_height', default_value='240'),

        # --- Visualisation ---
        DeclareLaunchArgument('vis_fps', default_value='5'),

        # --- Filter timing ---
        DeclareLaunchArgument('maxDelayBeforeDropping', default_value='0.2'),
        DeclareLaunchArgument('maxTimeCamInactive',     default_value='2.0'),

        # --- Runtime logging ---
        DeclareLaunchArgument('storeRuntimes', default_value='false'),

        # --- Filter config file (.info) ---
        DeclareLaunchArgument(
            'filter_config',
            default_value=config_file,
            description='Absolute path to the rovtio .info config file'
        ),
    ]

    # -------------------------------------------------------------------------
    # Node
    # -------------------------------------------------------------------------

    rovtio_node = Node(
        package='rovtio',
        executable='rovtio_node',
        name='rovtio',
        output='screen',
        parameters=[{
            'map_frame':    LaunchConfiguration('map_frame'),
            'world_frame':  LaunchConfiguration('world_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'imu_frame':    LaunchConfiguration('imu_frame'),

            'imu_topic':  LaunchConfiguration('imu_topic'),
            'cam0_topic': LaunchConfiguration('cam0_topic'),
            'cam1_topic': LaunchConfiguration('cam1_topic'),

            'cam0_offset': LaunchConfiguration('cam0_offset'),
            'cam1_offset': LaunchConfiguration('cam1_offset'),

            'resize_image':        LaunchConfiguration('resize_image'),
            'resize_image_width':  LaunchConfiguration('resize_image_width'),
            'resize_image_height': LaunchConfiguration('resize_image_height'),

            'vis_fps': LaunchConfiguration('vis_fps'),

            'maxDelayBeforeDropping': LaunchConfiguration('maxDelayBeforeDropping'),
            'maxTimeCamInactive':     LaunchConfiguration('maxTimeCamInactive'),

            'storeRuntimes': LaunchConfiguration('storeRuntimes'),

            'filter_config': LaunchConfiguration('filter_config'),
        }],
    )

    return LaunchDescription(declared_args + [rovtio_node])
