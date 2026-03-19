#!/usr/bin/env python3
"""Publish /pepper/odom derived from /gazebo/model_states with realistic noise.

Ground-truth pose comes from the gazebo_ros_state world plugin.  Gaussian
noise is added to match the real Pepper robot's wheel-odometry drift,
measured from UTS Magic Lab robots (same values as the original ROS 1
gazebo_model_velocity_plugin README):

  XY noise sigma  : 0.005 m / update (5 Hz model_states → ~0.025 m/s drift)
  Yaw noise sigma : 0.001 rad / update

A ground-truth topic (/pepper/odom_groundtruth) is also published without
noise for debugging and evaluation.
"""
import math
import random
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import geometry_msgs.msg

# Noise sigmas — calibrated to real Pepper hardware
_SIGMA_XY  = 0.005   # metres per model_states callback
_SIGMA_YAW = 0.001   # radians per model_states callback


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self._pub_noisy = self.create_publisher(Odometry, '/pepper/odom', 10)
        self._pub_gt    = self.create_publisher(Odometry, '/pepper/odom_groundtruth', 10)

        self._sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self._cb, 10)

        # Accumulated noisy odometry (integrated from ground-truth deltas + noise)
        self._noisy_x   = None   # initialised on first message
        self._noisy_y   = None
        self._noisy_yaw = None

        self._prev_gt_x   = None
        self._prev_gt_y   = None
        self._prev_gt_yaw = None

        self.get_logger().info('OdomPublisher ready (with odometry noise).')

    # ------------------------------------------------------------------
    def _cb(self, msg: ModelStates):
        try:
            idx = msg.name.index('pepper_MP')
        except ValueError:
            return

        pose  = msg.pose[idx]
        twist = msg.twist[idx]
        now   = self.get_clock().now().to_msg()

        # Ground-truth yaw
        q = pose.orientation
        gt_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        gt_x = pose.position.x
        gt_y = pose.position.y

        # Initialise noisy odom to ground-truth on first callback
        if self._noisy_x is None:
            self._noisy_x   = gt_x
            self._noisy_y   = gt_y
            self._noisy_yaw = gt_yaw
            self._prev_gt_x   = gt_x
            self._prev_gt_y   = gt_y
            self._prev_gt_yaw = gt_yaw

        # Delta in ground-truth world frame
        dx  = gt_x   - self._prev_gt_x
        dy  = gt_y   - self._prev_gt_y
        dya = gt_yaw - self._prev_gt_yaw
        # Wrap yaw delta to [-pi, pi]
        dya = math.atan2(math.sin(dya), math.cos(dya))

        self._prev_gt_x   = gt_x
        self._prev_gt_y   = gt_y
        self._prev_gt_yaw = gt_yaw

        # Integrate with noise (only add noise when actually moving)
        moving = abs(dx) > 1e-6 or abs(dy) > 1e-6 or abs(dya) > 1e-6
        if moving:
            self._noisy_x   += dx  + random.gauss(0, _SIGMA_XY)
            self._noisy_y   += dy  + random.gauss(0, _SIGMA_XY)
            self._noisy_yaw += dya + random.gauss(0, _SIGMA_YAW)
            self._noisy_yaw  = math.atan2(
                math.sin(self._noisy_yaw), math.cos(self._noisy_yaw))
        else:
            self._noisy_x   += dx
            self._noisy_y   += dy
            self._noisy_yaw += dya

        # Build noisy quaternion from yaw
        nqz = math.sin(self._noisy_yaw / 2.0)
        nqw = math.cos(self._noisy_yaw / 2.0)

        # --- Noisy odometry ---
        noisy = Odometry()
        noisy.header.stamp    = now
        noisy.header.frame_id = 'odom'
        noisy.child_frame_id  = 'base_footprint'
        noisy.pose.pose.position.x    = self._noisy_x
        noisy.pose.pose.position.y    = self._noisy_y
        noisy.pose.pose.position.z    = pose.position.z
        noisy.pose.pose.orientation.z = nqz
        noisy.pose.pose.orientation.w = nqw
        noisy.twist.twist = twist
        # Covariance (diagonal) matching noise sigmas
        noisy.pose.covariance[0]  = _SIGMA_XY  ** 2   # xx
        noisy.pose.covariance[7]  = _SIGMA_XY  ** 2   # yy
        noisy.pose.covariance[35] = _SIGMA_YAW ** 2   # yaw-yaw
        self._pub_noisy.publish(noisy)

        # --- Ground-truth odometry ---
        gt = Odometry()
        gt.header.stamp    = now
        gt.header.frame_id = 'odom'
        gt.child_frame_id  = 'base_footprint'
        gt.pose.pose  = pose
        gt.twist.twist = twist
        self._pub_gt.publish(gt)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
