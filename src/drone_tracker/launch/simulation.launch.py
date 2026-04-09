"""
simulation.launch.py
--------------------
Launches the full drone tracking simulation stack.

Usage (after building):
  ros2 launch drone_tracker simulation.launch.py

Optional args:
  use_gazebo:=true/false   (default false — runs in pure ROS mode with RViz only)
  world:=<path>            (Gazebo world file, default worlds/tracking_arena.world)
  rviz:=true/false         (default true)
  log_level:=debug/info    (default info)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction,
    IncludeLaunchDescription, LogInfo, TimerAction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


PKG = 'drone_tracker'


def generate_launch_description():
    pkg_share = FindPackageShare(PKG)

    # ── Launch arguments ──
    args = [
        DeclareLaunchArgument('use_gazebo', default_value='false',
                              description='Launch Gazebo simulator'),
        DeclareLaunchArgument('rviz',       default_value='true',
                              description='Launch RViz for visualisation'),
        DeclareLaunchArgument('log_level',  default_value='info',
                              description='ROS log level'),
        DeclareLaunchArgument('target_model', default_value='walking_actor',
                              description='Gazebo model name of the tracking target'),
        DeclareLaunchArgument('start_z',    default_value='5.0',
                              description='Initial drone altitude (metres)'),
    ]

    use_gazebo   = LaunchConfiguration('use_gazebo')
    use_rviz     = LaunchConfiguration('rviz')
    log_level    = LaunchConfiguration('log_level')
    target_model = LaunchConfiguration('target_model')
    start_z      = LaunchConfiguration('start_z')

    # ── Shared parameters ──
    shared_params = PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])

    # ── Node definitions ──
    flight_bridge = Node(
        package=PKG, executable='flight_bridge_node.py',
        name='flight_bridge',
        parameters=[shared_params, {'start_z': start_z}],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )

    detection_node = Node(
        package=PKG, executable='detection_node.py',
        name='detection',
        parameters=[shared_params, {'target_model': target_model}],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )

    tracker_node = Node(
        package=PKG, executable='tracker_node.py',
        name='tracker',
        parameters=[shared_params],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )

    pid_node = Node(
        package=PKG, executable='pid_controller_node.py',
        name='pid_controller',
        parameters=[shared_params],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )

    visualizer = Node(
        package=PKG, executable='visualizer_node.py',
        name='visualizer',
        parameters=[shared_params],
        output='screen',
    )

    # Static TF: world → map (identity)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
    )

    # RViz
    rviz_cfg = PathJoinSubstitution([pkg_share, 'config', 'tracking.rviz'])
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    # Gazebo (optional)
    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'tracking_arena.world'])
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        condition=IfCondition(use_gazebo),
    )

    # Start flight bridge first, then detectors after 1 second
    delayed_vision = TimerAction(period=1.0, actions=[detection_node, tracker_node])
    delayed_pid    = TimerAction(period=1.5, actions=[pid_node])

    return LaunchDescription(args + [
        LogInfo(msg='=== Drone Object Tracking Simulation ==='),
        LogInfo(msg=['Gazebo: ', use_gazebo, '  RViz: ', use_rviz]),
        gazebo,
        static_tf,
        flight_bridge,
        delayed_vision,
        delayed_pid,
        visualizer,
        rviz_node,
    ])
