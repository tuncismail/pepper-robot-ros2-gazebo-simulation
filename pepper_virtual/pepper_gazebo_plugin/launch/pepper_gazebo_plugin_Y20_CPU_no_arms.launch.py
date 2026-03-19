#!/usr/bin/env python3
"""Launch Pepper in Gazebo with an empty arena (naoFoot world).

Arguments:
  gui   -- show Gazebo gzclient window (default: false)
  rviz  -- launch RViz2 (default: false)
  arms  -- include arms and arm controllers (default: false)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gui = LaunchConfiguration('gui', default='false')
    rviz = LaunchConfiguration('rviz', default='false')
    arms = LaunchConfiguration('arms', default='false')

    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    pepper_description_pkg = get_package_share_directory('pepper_description')
    pepper_gazebo_pkg = get_package_share_directory('pepper_gazebo_plugin')

    world_file = os.path.join(pepper_gazebo_pkg, 'worlds', 'naoFoot.world')
    xacro_dir = os.path.join(
        pepper_description_pkg, 'urdf', 'pepper1.0_generated_urdf',
    )
    xacro_file = PythonExpression([
        "'", xacro_dir, "/pepper_robot_CPU.xacro'",
        " if '", arms, "'.lower() in ('true','1') else ",
        "'", xacro_dir, "/pepper_robot_CPU_no_arms.xacro'",
    ])

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'pepper_MP',
            '-topic', 'robot_description',
            '-x', '-0.5', '-y', '1', '-z', '0.05',
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=8.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        )]
    )

    head_controller_spawner = TimerAction(
        period=10.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['head_controller'],
            output='screen',
        )]
    )

    pelvis_controller_spawner = TimerAction(
        period=10.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['pelvis_controller'],
            output='screen',
        )]
    )

    laser_publisher = Node(
        package='pepper_gazebo_plugin',
        executable='laser_publisher.py',
        name='laser_publisher',
        output='screen',
    )

    odom_publisher = Node(
        package='pepper_gazebo_plugin',
        executable='odom_publisher.py',
        name='odom_publisher',
        output='screen',
    )

    # velocity_controller.py removed — the C++ libgazebo_ros_model_velocity
    # plugin (loaded via URDF) handles base velocity control directly in the
    # Gazebo physics loop.  Running both causes conflicts.

    left_arm_controller_spawner = TimerAction(
        period=10.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['left_arm_controller'],
            output='screen',
            condition=IfCondition(arms),
        )]
    )

    right_arm_controller_spawner = TimerAction(
        period=10.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['right_arm_controller'],
            output='screen',
            condition=IfCondition(arms),
        )]
    )

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

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false',
                              description='Launch Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch RViz2'),
        DeclareLaunchArgument('arms', default_value='false',
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
    ])
