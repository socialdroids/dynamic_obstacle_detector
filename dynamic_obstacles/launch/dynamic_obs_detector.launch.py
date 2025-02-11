import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, FindPackageShare, PathJoinSubstitution
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('frogbag', default_value='false'),

        # A outros editores: Pelo que eu entendi é desse frogbag ao qual não preciamos usar....
        # DeclareLaunchArgument('launch_prefix', default_value='', condition=UnlessCondition(LaunchConfiguration('debug'))),
        # DeclareLaunchArgument('launch_prefix', default_value='gdb -ex run --args', condition=IfCondition(LaunchConfiguration('debug'))),
        # DeclareLaunchArgument('scan_topic', default_value='scan_raw', condition=UnlessCondition(LaunchConfiguration('frogbag'))),
        # DeclareLaunchArgument('scan_topic', default_value='scan_filtered_drop', condition=IfCondition(LaunchConfiguration('frogbag'))),
        # launch.actions.GroupAction(
        #     actions=[
        #         Node(
        #             package='rosbag2_transport',
        #             executable='play',
        #             name='bagplayer',
        #             output='screen',
        #             arguments=['--clock', '/home/kenny/Downloads/UPO_pioneer_sensors_2014-04-29-11-36-22.bag'],
        #             condition=IfCondition(LaunchConfiguration('frogbag'))
        #         ),
        #         Node(
        #             package='laser_filters',
        #             executable='scan_to_scan_filter_chain',
        #             name='laser_filter',
        #             parameters=[{'use_sim_time': True}],
        #             remappings=[('scan', 'scanfront')],
        #             condition=IfCondition(LaunchConfiguration('frogbag'))
        #         ),
        #         Node(
        #             package='topic_tools',
        #             executable='drop',
        #             name='topic_drop',
        #             arguments=['scan_filtered', '3', '4'],
        #             condition=IfCondition(LaunchConfiguration('frogbag'))
        #         ),
        #         Node(
        #             package='rviz2',
        #             executable='rviz2',
        #             name='rviz',
        #             arguments=['-d', PathJoinSubstitution([FindPackageShare('dynamic_obstacle_detector'), 'launch', 'frogbag.rviz'])],
        #             condition=IfCondition(LaunchConfiguration('frogbag'))
        #         ),
        #     ]
        # ),
        
        # Node(
        #     package='gazebo_sfm_plugin',
        #     executable='tiago_pedestrians.launch',
        #     name='gazebo_sfm_plugin',
        #     condition=UnlessCondition(LaunchConfiguration('frogbag'))
        # ),
        # Lançamento do nó do dynamic_obstacle_detector;

        Node(
            package='dynamic_obstacle_detector',
            executable='dynamic_obstacle_detector_node',
            name='dynamic_obstacle_detector',
            output='screen',
            parameters=[
                {'input_scan_topic': LaunchConfiguration('base_scan_front_filtered')},
                {'odom_frame': 'odom'},
                {'cluster_max_distance_points': 0.6},
                {'cluster_min_points': 3},
                {'cluster_max_points': 40},
                {'min_vel_tracked': 0.37},
                {'max_vel_tracked': 1.8},
                {'max_tracked_distance': 0.55},
                {'max_tracked_sec': 0.8}
            ]
        ),

    ])
