# gazebo_model_velocity_plugin

A Gazebo Classic plugin for omnidirectional base velocity control with realistic motion dynamics. Combines the functionality of:

* **gazebo_ros_p3d** — Publishes odometry with separate Gaussian noise for XY and Yaw (noise is only applied when the robot is moving). Allows the odom frame to be `odom`.
* **ros_planar_move** — Drives the robot via Twist commands in Gazebo with velocity, acceleration, and jerk limits to approximate real robot dynamics. Automatically stops the robot if no commands are received within a configurable timeout. Similar to `diff_drive_controller` but with omnidirectional support and custom motion limiting.

> Originally developed by [awesomebytes](https://github.com/awesomebytes/gazebo_model_velocity_plugin). Ported to ROS 2 Humble for use with the Pepper robot simulation.

## Usage

```bash
ros2 launch pepper_gazebo_plugin pepper_gazebo_plugin_Y20_CPU_no_arms.launch.py
```

Command the robot:

```bash
ros2 topic pub --once /pepper/cmd_vel geometry_msgs/msg/Twist \
  '{"linear": {"x": 0.3, "y": 0.0}, "angular": {"z": 0.0}}'
```

Monitor the actual velocity sent to Gazebo (after applying limits):

```bash
ros2 topic echo /pepper/output_vel
```

## Configuration

The plugin is configured via URDF/xacro (see `pepperGazeboCPU.xacro`):

```xml
<gazebo>
  <plugin name="base_drive_controller" filename="libgazebo_ros_model_velocity.so">
    <robotNamespace>pepper</robotNamespace>
    <commandTopic>/pepper/cmd_vel</commandTopic>
    <outputVelocityTopic>/pepper/output_vel</outputVelocityTopic>
    <updateRate>50.0</updateRate>
    <commandTimeout>0.5</commandTimeout>
    <odometryTopic>/pepper/odom_plugin</odometryTopic>
    <odometryFrame>odom</odometryFrame>
    <odometryRate>20.0</odometryRate>
    <publishOdometryTf>true</publishOdometryTf>
    <robotBaseFrame>base_footprint</robotBaseFrame>
    <!-- Gaussian noise calibrated to real Pepper odometry -->
    <gaussianNoiseXY>0.02</gaussianNoiseXY>
    <gaussianNoiseYaw>0.02645</gaussianNoiseYaw>
    <!-- Motion limits -->
    <linearVelocityLimit>0.55</linearVelocityLimit>
    <angularVelocityLimit>2.0</angularVelocityLimit>
    <linearAccelerationLimit>0.44</linearAccelerationLimit>
    <angularAccelerationLimit>2.4</angularAccelerationLimit>
    <linearJerkLimit>4.5</linearJerkLimit>
    <angularJerkLimit>45.0</angularJerkLimit>
  </plugin>
</gazebo>
```
