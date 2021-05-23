
import os
from ament_index_python.packages import get_package_share_directory , get_search_paths
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription


namespace_name = 'smartmbot_1'

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('smart_mbot_pkg'),
         'config',
          namespace_name+'__Writing_DC_Motor.yaml'
        )

    return LaunchDescription([
        Node(
            package='smart_mbot_pkg',
            namespace= namespace_name,
            executable='Reading_SPI_ADC',
            name='Reading_SPI_ADC'
        ),

        Node(
            package='smart_mbot_pkg',
            namespace= namespace_name,
            executable='Writing_DC_Motor',
            name='Writing_DC_Motor',
            output='screen',
            parameters = [config]
        ),

        Node(
            package='smart_mbot_pkg',
            namespace= namespace_name,
            executable='Reading_I2C_ToF',
            name='Reading_I2C_ToF'
        ),

        Node(
            package='smart_mbot_pkg',
            namespace=namespace_name,
            executable='Writing_WB2813b_RGB_STRIP',
            name='Writing_WB2813b_RGB_STRIP'
        ),

        Node(
            package='smart_mbot_pkg',
            namespace=namespace_name,
            executable='Writing_GPIO_SMD5050_LED',
            name='Writing_GPIO_SMD5050_LED'
        ),

        Node(
            package='v4l2_camera',
            namespace=namespace_name,
            executable='v4l2_camera_node',
            name='v4l2_camera_node'
        )
    ])
