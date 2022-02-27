import os
from ament_index_python.packages import get_package_share_directory , get_search_paths
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription, Action
import launch


def generate_launch_description():  
    # For vicon
    hostname = '192.168.50.22'
    buffer_size = 256
    topic_namespace = 'vicon'
    
    # For robot platforms
    robot_controller = 'line_tracing_controller'
    robot_speed = 50
    adc_threshold = 1.5 # for line sensors to detect black lines

    vicon_reading_node = Node(
            package='vicon_receiver', executable='vicon_client', output='screen',
            parameters=[{'hostname': hostname, 'buffer_size': buffer_size, 'namespace': topic_namespace}]
        )

    smartmbot_node_1 = Node(
        package='affective_smartmbot_pkg', 
        executable=robot_controller, 
        name='smartmbot_1_controller',
        output='screen',
        parameters=[{'robot_name': 'smartmbot_4',
                    'moving_speed': robot_speed, 
                    'adc_threshold': adc_threshold}]
        )  
    data_recording = launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', "go_to_goal_3",  "/vicon/smartmbot_4/smartmbot_4","/vicon/cup/cup" ],
            output='screen'
        )
       
   
    return LaunchDescription([vicon_reading_node, smartmbot_node_1, data_recording
    ])
