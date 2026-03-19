#!/usr/bin/env python3
import rclpy
import socket
import json
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

# --- CONFIGURATION (front laser only) ---
UDP_IP            = "0.0.0.0"     # Listen on all interfaces
UDP_PORT          = 5006
ANGLE_MIN         = -math.pi/6    # -30°
ANGLE_MAX         =  math.pi/6    # +30°
PHYS_SEGMENTS     = 15            # Physical beam count
PUBLISH_SEGMENTS  = 60            # Interpolated beam count to publish
# --- END CONFIGURATION ---


def main():
    rclpy.init()
    node = rclpy.create_node("pepper_laser_receiver")
    pub = node.create_publisher(LaserScan, "/real_scan", 1)

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)  # non-blocking with timeout so rclpy.ok() can be checked
    try:
        sock.bind((UDP_IP, UDP_PORT))
    except socket.error as e:
        node.get_logger().fatal(
            "Socket bind error %s:%d – %s" % (UDP_IP, UDP_PORT, str(e)))
        sock.close()
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info(
        "Listening for front-laser (%d phys -> %d pub beams) on UDP %s:%d" %
        (PHYS_SEGMENTS, PUBLISH_SEGMENTS, UDP_IP, UDP_PORT))

    while rclpy.ok():
        try:
            data, _ = sock.recvfrom(16_384)
        except socket.timeout:
            # Check shutdown and retry
            rclpy.spin_once(node, timeout_sec=0.0)
            continue
        except Exception as e:
            node.get_logger().error("UDP recv error: %s" % str(e))
            continue

        try:
            parsed   = json.loads(data.decode("utf-8"))
            vectors  = parsed.get("vectors", [])

            # Validate beam count
            if len(vectors) != PHYS_SEGMENTS:
                node.get_logger().warning(
                    "Received %d physical beams, expected %d" %
                    (len(vectors), PHYS_SEGMENTS))
                continue

            # 1) Compute physical ranges
            phys_ranges = np.array([math.hypot(float(x), float(y))
                                    for x, y in vectors])

            # 2) Build angle axes
            phys_angles = np.linspace(ANGLE_MIN, ANGLE_MAX, PHYS_SEGMENTS)
            pub_angles  = np.linspace(ANGLE_MIN, ANGLE_MAX, PUBLISH_SEGMENTS)

            # 3) Linearly interpolate to target beam count
            interp_ranges = np.interp(pub_angles, phys_angles, phys_ranges)

            # 4) Build LaserScan message
            scan = LaserScan()
            scan.header.stamp    = node.get_clock().now().to_msg()
            scan.header.frame_id = "base_footprint"

            scan.angle_min       = ANGLE_MIN
            scan.angle_max       = ANGLE_MAX
            scan.angle_increment = (ANGLE_MAX - ANGLE_MIN) / (PUBLISH_SEGMENTS - 1)
            scan.time_increment  = 0.0
            scan.scan_time       = 0.1
            scan.range_min       = 0.1
            scan.range_max       = 5.0

            scan.ranges      = interp_ranges.tolist()
            scan.intensities = [1.0] * PUBLISH_SEGMENTS

            pub.publish(scan)

        except json.JSONDecodeError as e:
            node.get_logger().warning("JSON decode error: %s" % str(e))
        except Exception as e:
            node.get_logger().error("UDP processing error: %s" % str(e))

        rclpy.spin_once(node, timeout_sec=0.0)

    sock.close()
    node.get_logger().info("Laser receiver node shut down.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
