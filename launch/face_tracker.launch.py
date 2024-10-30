from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    detect_only = LaunchConfiguration('detect_only')
    detect_only_dec = DeclareLaunchArgument(
        'detect_only',
        default_value='false',
        description='Doesn\'t run the follow component. Useful for just testing the detections.')
    
    follow_only = LaunchConfiguration('follow_only')
    follow_only_dec = DeclareLaunchArgument(
        'follow_only',
        default_value='false',
        description='Doesn\'t run the detect component. Useful for testing just the following. (e.g. with manually published detections)')
    
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_dec = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Enables sim time for the follow node.')
    
    image_topic = LaunchConfiguration('image_topic')
    image_topic_dec = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw/decompressed',
        description='The name of the input image topic.')

    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    cmd_vel_topic_dec = DeclareLaunchArgument(
    'cmd_vel_topic',
    default_value='/cmd_vel',
    description='The name of the output command vel topic.')

    decompress_node = Node(
            package='image_transport',
            executable='republish',
            name = "decompress_node",
            arguments=['compressed'],
            remappings=[
                    ('in/compressed','camera/image_raw/compressed'), 
                    ('out','/camera/image_raw/decompressed'),
            ]
         )   

    detect_node = Node(
            package='face_tracker',
            executable='detect_face',
            #parameters=[params_file],
            remappings=[('/image_in',image_topic)],
            condition=UnlessCondition(follow_only)
         )

    follow_node = Node(
            package='face_tracker',
            executable='follow_face',
            #parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel',cmd_vel_topic)],
            condition=UnlessCondition(detect_only)
         )




    return LaunchDescription([
        decompress_node,
        detect_only_dec,
        follow_only_dec,
        use_sim_time_dec,
        image_topic_dec,
        cmd_vel_topic_dec,
        detect_node,
        follow_node,    
    ])