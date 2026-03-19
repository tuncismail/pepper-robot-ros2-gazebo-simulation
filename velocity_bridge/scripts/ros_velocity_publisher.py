#!/usr/bin/env python3

import rclpy
import socket
import json
from geometry_msgs.msg import Twist

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

TARGET_IP = None
TARGET_PORT = None


def cmd_vel_callback(msg):
    payload = {
        "x": msg.linear.x,
        "y": msg.linear.y,
        "theta": msg.angular.z
    }
    try:
        sock.sendto(json.dumps(payload).encode("utf-8"), (TARGET_IP, TARGET_PORT))
    except Exception as e:
        node.get_logger().error("Failed to send UDP packet: %s" % str(e))


def main():
    global TARGET_IP, TARGET_PORT, node
    rclpy.init()
    node = rclpy.create_node("ros_velocity_publisher")
    node.declare_parameter("target_ip", "127.0.0.1")
    node.declare_parameter("target_port", 5005)
    TARGET_IP   = node.get_parameter("target_ip").value
    TARGET_PORT = node.get_parameter("target_port").value
    node.create_subscription(Twist, "/pepper/cmd_vel", cmd_vel_callback, 1)
    node.get_logger().info(
        "Publishing /pepper/cmd_vel via UDP to %s:%d" % (TARGET_IP, TARGET_PORT))
    try:
        rclpy.spin(node)
    finally:
        sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
