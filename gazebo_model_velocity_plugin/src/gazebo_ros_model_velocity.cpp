/*
 * Copyright 2018 Sammy Pfeiffer, The Magic Lab, University of Technology Sydney
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Simple model controller that uses a twist message to set
 *       velocities on a robot, resulting in motion. Also publishes
 *       odometry. Based on the
 *       gazebo_ros_force_based_move and ros_control speed_limit
 *       to implement velocity, acceleration and jerk limits.
 * Author: Sammy Pfeiffer
 * Date: 24 December 2018
 * ROS 2 Humble port: 2024
 */

#include <math.h>
#include <functional>
#include <gazebo_model_velocity_plugin/gazebo_ros_model_velocity.h>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{

  GazeboRosModelVelocity::GazeboRosModelVelocity() {}

  GazeboRosModelVelocity::~GazeboRosModelVelocity() {}

  // Load the controller
  void GazeboRosModelVelocity::Load(physics::ModelPtr parent,
      sdf::ElementPtr sdf)
  {
    parent_ = parent;

    /* Parse parameters */
    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace"))
    {
      // default empty namespace
    }
    else
    {
      robot_namespace_ =
        sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    command_topic_ = "cmd_vel";
    if (!sdf->HasElement("commandTopic"))
    {
      // use default
    }
    else
    {
      command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
    }

    output_vel_topic_ = "output_vel";
    if (!sdf->HasElement("outputVelocityTopic"))
    {
      // use default
    }
    else
    {
      output_vel_topic_ = sdf->GetElement("outputVelocityTopic")->Get<std::string>();
    }

    update_rate_ = 20.0;
    if (!sdf->HasElement("updateRate"))
    {
      // use default
    }
    else
    {
      update_rate_ = sdf->GetElement("updateRate")->Get<double>();
    }

    command_timeout_ = 0.5;
    if (!sdf->HasElement("commandTimeout"))
    {
      // use default
    }
    else
    {
      command_timeout_ = sdf->GetElement("commandTimeout")->Get<double>();
    }

    limiter_lin_ = gazebo_model_velocity_plugin::SpeedLimiter();
    limiter_ang_ = gazebo_model_velocity_plugin::SpeedLimiter();

    // Give some reasonable default
    limiter_lin_.max_velocity = 100.0;
    if (!sdf->HasElement("linearVelocityLimit"))
    {
      // use default
    }
    else
    {
      limiter_lin_.max_velocity = sdf->GetElement("linearVelocityLimit")->Get<double>();
      limiter_lin_.min_velocity = - limiter_lin_.max_velocity;
    }

    // Give some reasonable default
    limiter_ang_.max_velocity = 10.0;
    if (!sdf->HasElement("angularVelocityLimit"))
    {
      // use default
    }
    else
    {
      limiter_ang_.max_velocity = sdf->GetElement("angularVelocityLimit")->Get<double>();
      limiter_ang_.min_velocity = - limiter_ang_.max_velocity;
    }

    // Give some reasonable default
    limiter_lin_.max_acceleration = 10.0;
    if (!sdf->HasElement("linearAccelerationLimit"))
    {
      // use default
    }
    else
    {
      limiter_lin_.max_acceleration = sdf->GetElement("linearAccelerationLimit")->Get<double>();
      limiter_lin_.min_acceleration = - limiter_lin_.max_acceleration;
    }

    // Give some reasonable default
    limiter_ang_.max_acceleration = 10.0;
    if (!sdf->HasElement("angularAccelerationLimit"))
    {
      // use default
    }
    else
    {
      limiter_ang_.max_acceleration = sdf->GetElement("angularAccelerationLimit")->Get<double>();
      limiter_ang_.min_acceleration = - limiter_ang_.max_acceleration;
    }

    // Give some reasonable default
    limiter_lin_.max_jerk = 100.0;
    if (!sdf->HasElement("linearJerkLimit"))
    {
      // use default
    }
    else
    {
      limiter_lin_.max_jerk = sdf->GetElement("linearJerkLimit")->Get<double>();
      limiter_lin_.min_jerk = - limiter_lin_.max_jerk;
    }

    // Give some reasonable default
    limiter_ang_.max_jerk = 1000.0;
    if (!sdf->HasElement("angularJerkLimit"))
    {
      // use default
    }
    else
    {
      limiter_ang_.max_jerk = sdf->GetElement("angularJerkLimit")->Get<double>();
      limiter_ang_.min_jerk = - limiter_ang_.max_jerk;
    }

    odometry_topic_ = "odom";
    if (!sdf->HasElement("odometryTopic"))
    {
      // use default
    }
    else
    {
      odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    odometry_frame_ = "odom";
    if (!sdf->HasElement("odometryFrame"))
    {
      // use default
    }
    else
    {
      odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    odometry_rate_ = 20.0;
    if (!sdf->HasElement("odometryRate"))
    {
      // use default
    }
    else
    {
      odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
    }

    publish_odometry_tf_ = true;
    if (!sdf->HasElement("publishOdometryTf"))
    {
      // use default
    }
    else
    {
      publish_odometry_tf_ = sdf->GetElement("publishOdometryTf")->Get<bool>();
    }

    robot_base_frame_ = "base_footprint";
    if (!sdf->HasElement("robotBaseFrame"))
    {
      // use default
    }
    else
    {
      robot_base_frame_ = sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    if (!sdf->HasElement("gaussianNoiseXY"))
    {
      gaussian_noise_xy_ = 0;
    }
    else
    {
      gaussian_noise_xy_ = sdf->GetElement("gaussianNoiseXY")->Get<double>();
    }

    if (!sdf->HasElement("gaussianNoiseYaw"))
    {
      gaussian_noise_yaw_ = 0;
    }
    else
    {
      gaussian_noise_yaw_ = sdf->GetElement("gaussianNoiseYaw")->Get<double>();
    }

    seed = 0;

    // Enable the limits to be applied
    limiter_lin_.has_velocity_limits = true;
    limiter_lin_.has_acceleration_limits = true;
    limiter_lin_.has_jerk_limits = true;
    limiter_ang_.has_velocity_limits = true;
    limiter_ang_.has_acceleration_limits = true;
    limiter_ang_.has_jerk_limits = true;

    current_cmd_ = geometry_msgs::msg::Twist();
    last_cmd0_   = geometry_msgs::msg::Twist();
    last_cmd1_   = geometry_msgs::msg::Twist();

    // Initialize ROS 2 node directly (gazebo_ros::Node::Get fails for
    // model plugins loaded from dynamically spawned URDF entities)
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    // Strip leading slash from namespace for rclcpp::Node (it re-adds it)
    std::string ns = robot_namespace_;
    if (!ns.empty() && ns[0] == '/') ns = ns.substr(1);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.append_parameter_override("use_sim_time", true);
    ros_node_ = std::make_shared<rclcpp::Node>("pepper_velocity_controller", ns, node_options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(ros_node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    RCLCPP_INFO_STREAM(ros_node_->get_logger(),
      "GazeboRosModelVelocity Plugin (" << robot_namespace_ << ") has started!");

    last_command_time_        = ros_node_->now();
    last_velocity_update_time_ = ros_node_->now();
    last_odom_publish_time_   = ros_node_->now();

    // Subscribe to cmd_vel
    vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      command_topic_, 1,
      std::bind(&GazeboRosModelVelocity::cmdVelCallback, this,
                std::placeholders::_1));

    output_vel_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Twist>(
      output_vel_topic_, 1);

    odometry_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      odometry_topic_, 1);

    if (publish_odometry_tf_)
    {
      transform_broadcaster_ =
        std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);
    }

    // Reset accumulated odometry
    odom_x_   = 0.0;
    odom_y_   = 0.0;
    odom_yaw_ = 0.0;

    // Listen to the update event (broadcast every simulation iteration)
    update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboRosModelVelocity::UpdateChild, this));
  }


  // Update the controller
  void GazeboRosModelVelocity::UpdateChild()
  {
    std::lock_guard<std::mutex> scoped_lock(lock);
    rclcpp::Time tnow = ros_node_->now();
    rclcpp::Duration dt = tnow - last_velocity_update_time_;
    geometry_msgs::msg::Twist cmd = current_cmd_;

    // If we stop receiving commands, stop the robot
    if ((tnow - last_command_time_) > rclcpp::Duration::from_seconds(command_timeout_))
    {
      cmd.linear.x  = 0.0;
      cmd.linear.y  = 0.0;
      cmd.angular.z = 0.0;
    }

    // Rate-limit the velocity limiting / publishing logic
    if (dt > rclcpp::Duration::from_seconds(1.0 / update_rate_))
    {
      // Apply limits
      double x_lim_factor     = limiter_lin_.limit(cmd.linear.x,  last_cmd0_.linear.x,  last_cmd1_.linear.x,  dt.seconds());
      double y_lim_factor     = limiter_lin_.limit(cmd.linear.y,  last_cmd0_.linear.y,  last_cmd1_.linear.y,  dt.seconds());
      double theta_lim_factor = limiter_ang_.limit(cmd.angular.z, last_cmd0_.angular.z, last_cmd1_.angular.z, dt.seconds());

      last_cmd1_ = last_cmd0_;
      last_cmd0_ = cmd;

      if (x_lim_factor != 1.0 || y_lim_factor != 1.0 || theta_lim_factor != 1.0)
      {
        RCLCPP_DEBUG_STREAM(ros_node_->get_logger(),
          "Limiting factors: "
          "\nx:     " << x_lim_factor <<
          "\ny:     " << y_lim_factor <<
          "\ntheta: " << theta_lim_factor);
      }

      output_vel_pub_->publish(cmd);
      last_velocity_update_time_ = tnow;
    }

    // Apply velocity EVERY physics step to overcome wheel friction.
    // The Gazebo API only supports world-frame velocity, so rotate
    // body-frame cmd_vel into the world frame using the current yaw.
    ignition::math::Pose3d pose = parent_->RelativePose();
    double yaw = pose.Rot().Yaw();

    // Standard 2D rotation matrix: body-frame -> world-frame
    double x_vel_cmd = cmd.linear.x * cos(yaw) - cmd.linear.y * sin(yaw);
    double y_vel_cmd = cmd.linear.x * sin(yaw) + cmd.linear.y * cos(yaw);

    parent_->SetLinearVel(ignition::math::Vector3d(x_vel_cmd, y_vel_cmd, 0.0));
    parent_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, cmd.angular.z));

    if (odometry_rate_ > 0.0)
    {
      double seconds_since_last_update =
        (tnow - last_odom_publish_time_).seconds();
      if (seconds_since_last_update > (1.0 / odometry_rate_))
      {
        publishOdometry(seconds_since_last_update);
        last_odom_publish_time_ = tnow;
      }
    }

    // Subscriptions are processed by the background spin thread.
  }

  // Finalize the controller
  void GazeboRosModelVelocity::FiniChild()
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  void GazeboRosModelVelocity::cmdVelCallback(
      const geometry_msgs::msg::Twist::SharedPtr cmd_msg)
  {
    std::lock_guard<std::mutex> scoped_lock(lock);
    current_cmd_ = *cmd_msg;

    RCLCPP_DEBUG_STREAM(ros_node_->get_logger(),
      "Updating command:\n"
      "x: "   << cmd_msg->linear.x  <<
      "\ny: "  << cmd_msg->linear.y  <<
      "\nyaw: " << cmd_msg->angular.z);

    last_command_time_ = ros_node_->now();
  }

  // Utility for adding noise
  double GazeboRosModelVelocity::GaussianKernel(double mu, double sigma)
  {
    // using Box-Muller transform to generate two independent standard
    // normally distributed normal variables see wikipedia

    // normalized uniform random variable
    double U = static_cast<double>(rand_r(&this->seed)) /
               static_cast<double>(RAND_MAX);

    // normalized uniform random variable
    double V = static_cast<double>(rand_r(&this->seed)) /
               static_cast<double>(RAND_MAX);

    double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);

    // scale to our mu and sigma
    X = sigma * X + mu;
    return X;
  }

  void GazeboRosModelVelocity::publishOdometry(double step_time)
  {
    rclcpp::Time current_time = ros_node_->now();
    std::string odom_frame          = odometry_frame_;
    std::string base_footprint_frame = robot_base_frame_;

    ignition::math::Vector3d angular_vel = parent_->RelativeAngularVel();
    ignition::math::Vector3d linear_vel  = parent_->RelativeLinearVel();

    double vx  = linear_vel.X()  + GaussianKernel(0, gaussian_noise_xy_);
    double vy  = linear_vel.Y()  + GaussianKernel(0, gaussian_noise_xy_);
    double vth = angular_vel.Z() + GaussianKernel(0, gaussian_noise_yaw_);

    // If we are actually not moving, don't add noise
    if (last_cmd0_.linear.x  == 0.0) vx  = 0.0;
    if (last_cmd0_.linear.y  == 0.0) vy  = 0.0;
    if (last_cmd0_.angular.z == 0.0) vth = 0.0;

    odom_.twist.twist.linear.x  = vx;
    odom_.twist.twist.linear.y  = vy;
    odom_.twist.twist.angular.z = vth;

    // Integrate position
    double delta_x   = (vx * cos(odom_yaw_) - vy * sin(odom_yaw_)) * step_time;
    double delta_y   = (vx * sin(odom_yaw_) + vy * cos(odom_yaw_)) * step_time;
    double delta_th  = vth * step_time;

    odom_x_   += delta_x;
    odom_y_   += delta_y;
    odom_yaw_ += delta_th;

    // Build pose quaternion from yaw
    double cy = cos(odom_yaw_ * 0.5);
    double sy = sin(odom_yaw_ * 0.5);

    odom_.pose.pose.position.x    = odom_x_;
    odom_.pose.pose.position.y    = odom_y_;
    odom_.pose.pose.position.z    = 0.0;
    odom_.pose.pose.orientation.x = 0.0;
    odom_.pose.pose.orientation.y = 0.0;
    odom_.pose.pose.orientation.z = sy;
    odom_.pose.pose.orientation.w = cy;

    odom_.header.stamp     = current_time;
    odom_.header.frame_id  = odom_frame;
    odom_.child_frame_id   = base_footprint_frame;

    if (transform_broadcaster_)
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp     = current_time;
      t.header.frame_id  = odom_frame;
      t.child_frame_id   = base_footprint_frame;
      t.transform.translation.x = odom_x_;
      t.transform.translation.y = odom_y_;
      t.transform.translation.z = 0.0;
      t.transform.rotation.x    = 0.0;
      t.transform.rotation.y    = 0.0;
      t.transform.rotation.z    = sy;
      t.transform.rotation.w    = cy;
      transform_broadcaster_->sendTransform(t);
    }

    odom_.pose.covariance[0]  = 0.001;
    odom_.pose.covariance[7]  = 0.001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;

    if (std::abs(angular_vel.Z()) < 0.0001)
    {
      odom_.pose.covariance[35] = 0.01;
    }
    else
    {
      odom_.pose.covariance[35] = 100.0;
    }

    odom_.twist.covariance[0]  = 0.001;
    odom_.twist.covariance[7]  = 0.001;
    odom_.twist.covariance[14] = 0.001;
    odom_.twist.covariance[21] = 1000000000000.0;
    odom_.twist.covariance[28] = 1000000000000.0;

    if (std::abs(angular_vel.Z()) < 0.0001)
    {
      odom_.twist.covariance[35] = 0.01;
    }
    else
    {
      odom_.twist.covariance[35] = 100.0;
    }

    odometry_pub_->publish(odom_);
  }


  GZ_REGISTER_MODEL_PLUGIN(GazeboRosModelVelocity)
}
