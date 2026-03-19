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
 * Desc: Simple model controller that uses a twist message to exert
 *       forces on a robot, resulting in motion. Based on the
 *       planar_move plugin by Piyush Khandelwal.
 * Author: Stefan Kohlbrecher, Sammy Pfeiffer
 * Date: 06 August 2015, 21 December 2018
 * ROS 2 Humble port: 2024
 */

#ifndef GAZEBO_ROS_MODEL_VELOCITY_HH
#define GAZEBO_ROS_MODEL_VELOCITY_HH

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gazebo_model_velocity_plugin/speed_limiter.h>

namespace gazebo {

  class GazeboRosModelVelocity : public ModelPlugin {

    public:
      GazeboRosModelVelocity();
      ~GazeboRosModelVelocity();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    protected:
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      physics::ModelPtr parent_;
      event::ConnectionPtr update_connection_;

      rclcpp::Node::SharedPtr ros_node_;
      rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
      std::thread spin_thread_;

      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_vel_pub_;

      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
      std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
      nav_msgs::msg::Odometry odom_;

      // Accumulated odometry transform (translation + rotation)
      double odom_x_{0.0};
      double odom_y_{0.0};
      double odom_yaw_{0.0};

      rclcpp::Time last_odom_publish_time_;
      double odometry_rate_;
      bool publish_odometry_tf_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;

      /// Gaussian noise
      double gaussian_noise_xy_;
      double gaussian_noise_yaw_;
      unsigned int seed;

      /// Gaussian noise generator
      double GaussianKernel(double mu, double sigma);

      void publishOdometry(double step_time);

      std::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string output_vel_topic_;
      double update_rate_;
      double command_timeout_;

      // command velocity callback
      void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_msg);

      // Latest command
      geometry_msgs::msg::Twist current_cmd_;
      geometry_msgs::msg::Twist last_cmd0_;
      geometry_msgs::msg::Twist last_cmd1_;

      gazebo_model_velocity_plugin::SpeedLimiter limiter_lin_;
      gazebo_model_velocity_plugin::SpeedLimiter limiter_ang_;

      rclcpp::Time last_velocity_update_time_;
      rclcpp::Time last_command_time_;

  };

}

#endif /* end of include guard: GAZEBO_ROS_MODEL_VELOCITY_HH */
