#!/usr/bin/env python3
"""Launch Pepper in Gazebo with Nav2 (SLAM Toolbox + DWB planner).

Usage:
  # SLAM (build map while navigating)
  ros2 launch pepper_gazebo_plugin pepper_navigation.launch.py

  # Localisation against existing map
  ros2 launch pepper_gazebo_plugin pepper_navigation.launch.py \
      use_slam:=false map:=/path/to/map.yaml

Arguments:
  gui         – show Gazebo gzclient window (default: false)
  world       – Gazebo .world file (default: naoFoot.world)
  use_slam    – run slam_toolbox instead of AMCL (default: true)
  map         – map YAML for AMCL mode (ignored when use_slam:=true)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gui      = LaunchConfiguration('gui',      default='false')
    rviz     = LaunchConfiguration('rviz',     default='false')
    world_lc = LaunchConfiguration('world',    default='naoFoot.world')
    use_slam = LaunchConfiguration('use_slam', default='true')
    map_yaml = LaunchConfiguration('map',      default='')
    arms     = LaunchConfiguration('arms',     default='false')

    gazebo_ros_pkg        = get_package_share_directory('gazebo_ros')
    pepper_description_pkg = get_package_share_directory('pepper_description')
    pepper_gazebo_pkg     = get_package_share_directory('pepper_gazebo_plugin')
    nav2_bringup_pkg      = get_package_share_directory('nav2_bringup')

    nav2_params_file = os.path.join(pepper_gazebo_pkg, 'config', 'nav2_params.yaml')
    xacro_dir = os.path.join(
        pepper_description_pkg, 'urdf', 'pepper1.0_generated_urdf',
    )
    xacro_file = PythonExpression([
        "'", xacro_dir, "/pepper_robot_CPU.xacro'",
        " if '", arms, "'.lower() in ('true','1') else ",
        "'", xacro_dir, "/pepper_robot_CPU_no_arms.xacro'",
    ])

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # ── Gazebo ───────────────────────────────────────────────────────────────
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': [os.path.join(pepper_gazebo_pkg, 'worlds/'), world_lc],
            'verbose': 'false',
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui),
    )

    # ── Robot description & spawn ─────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'pepper_MP', '-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.05'],
        output='screen',
    )

    # ── Controllers ───────────────────────────────────────────────────────────
    joint_state_broadcaster_spawner = TimerAction(period=8.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'], output='screen',
    )])

    # ── Pepper sensor nodes ───────────────────────────────────────────────────
    laser_publisher     = Node(package='pepper_gazebo_plugin',
                               executable='laser_publisher.py',
                               name='laser_publisher', output='screen',
                               parameters=[{'use_sim_time': True}])
    odom_publisher      = Node(package='pepper_gazebo_plugin',
                               executable='odom_publisher.py',
                               name='odom_publisher', output='screen',
                               parameters=[{'use_sim_time': True}])
    # velocity_controller.py removed — the C++ libgazebo_ros_model_velocity
    # plugin (loaded via URDF) handles base velocity control directly in the
    # Gazebo physics loop.  Running both causes conflicts.

    head_controller_spawner = TimerAction(period=10.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['head_controller'], output='screen',
    )])
    pelvis_controller_spawner = TimerAction(period=10.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['pelvis_controller'], output='screen',
    )])

    left_arm_controller_spawner = TimerAction(period=10.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['left_arm_controller'], output='screen',
        condition=IfCondition(arms),
    )])
    right_arm_controller_spawner = TimerAction(period=10.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['right_arm_controller'], output='screen',
        condition=IfCondition(arms),
    )])

    rviz_config = os.path.join(pepper_gazebo_pkg, 'config', 'pepper_sensors_rviz2.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen',
    )

    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    # ── SLAM Toolbox (async – tolerant of sim-time jitter in Docker) ─────
    nav2_slam = TimerAction(period=15.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': nav2_params_file,
        }.items(),
    )], condition=IfCondition(use_slam))

    # ── Nav2 navigation stack (delayed to let Gazebo + robot fully start) ─
    nav2_navigation = TimerAction(period=15.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'use_composition': 'False',
        }.items(),
    )])

    # ── Nav2 AMCL localisation (when not using SLAM) ─────────────────────
    nav2_amcl = TimerAction(period=15.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_yaml,
            'params_file': nav2_params_file,
            'use_composition': 'False',
        }.items(),
    )], condition=UnlessCondition(use_slam))

    # ── Relay Nav2 /cmd_vel → /pepper/cmd_vel ───────────────────────────────
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        parameters=[{'use_sim_time': True}],
        arguments=['/cmd_vel', '/pepper/cmd_vel'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui',      default_value='false',
                              description='Show Gazebo gzclient'),
        DeclareLaunchArgument('world',    default_value='naoFoot.world',
                              description='World file name (relative to worlds/)'),
        DeclareLaunchArgument('use_slam', default_value='true',
                              description='Run SLAM Toolbox (true) or AMCL (false)'),
        DeclareLaunchArgument('map',      default_value='',
                              description='Map YAML path for AMCL mode'),
        DeclareLaunchArgument('rviz',     default_value='false',
                              description='Launch RViz2'),
        DeclareLaunchArgument('arms',     default_value='false',
                              description='Include arms (true) or no arms (false)'),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        head_controller_spawner,
        pelvis_controller_spawner,
        left_arm_controller_spawner,
        right_arm_controller_spawner,
        laser_publisher,
        odom_publisher,
        rviz_node,
        nav2_slam,
        nav2_navigation,
        nav2_amcl,
        cmd_vel_relay,
    ])
