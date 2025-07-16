from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('datavideo_bc50_driver')
    calib = os.path.join(pkg_dir, 'config', 'datavideo_bc50.yaml')

    return LaunchDescription([
        Node(
            package='datavideo_bc50_driver',
            executable='bc50_controller',
            name='bc50_controller',
            output='screen',
            parameters=[{
                'camera_info_url': f'file://{calib}',
                # 'publish_rate_hz': 10.0, 
                'frame_id': 'camera_frame'
            }]
        ),
    ])
