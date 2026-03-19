#!/usr/bin/env python3
from math import atan2, cos, radians, sin, sqrt, degrees, isnan
from copy import deepcopy

import rclpy
import rclpy.node
import rclpy.time
import tf2_ros
from tf2_ros import TransformListener

import message_filters
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points, create_cloud_xyz32
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3, Quaternion

import sensor_msgs_py.point_cloud2 as pc2

import numpy as np

"""
Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
With LaserProject implementation borrowed from the laser_geometry package.
ROS 2 Humble port: 2024
"""


class LaserProjection:
    """
    A class to Project Laser Scan
    This class will project laser scans into point clouds. It caches
    unit vectors between runs (provided the angular resolution of
    your scanner is not changing) to avoid excess computation.
    """

    LASER_SCAN_INVALID = -1.0
    LASER_SCAN_MIN_RANGE = -2.0
    LASER_SCAN_MAX_RANGE = -3.0

    class ChannelOption:
        NONE = 0x00
        INTENSITY = 0x01
        INDEX = 0x02
        DISTANCE = 0x04
        TIMESTAMP = 0x08
        VIEWPOINT = 0x10
        DEFAULT = (INTENSITY | INDEX)

    def __init__(self):
        self.__angle_min = 0.0
        self.__angle_max = 0.0
        self.__cos_sin_map = np.array([[]])

    def projectLaser(self, scan_in,
                     range_cutoff=-1.0, channel_options=ChannelOption.DEFAULT):
        return self.__projectLaser(scan_in, range_cutoff, channel_options)

    def __projectLaser(self, scan_in, range_cutoff, channel_options):
        N = len(scan_in.ranges)
        ranges = np.array(scan_in.ranges)

        if (self.__cos_sin_map.shape[1] != N or
                self.__angle_min != scan_in.angle_min or
                self.__angle_max != scan_in.angle_max):

            self.__angle_min = scan_in.angle_min
            self.__angle_max = scan_in.angle_max

            angles = scan_in.angle_min + np.arange(N) * scan_in.angle_increment
            self.__cos_sin_map = np.array([np.cos(angles), np.sin(angles)])

        output = ranges * self.__cos_sin_map

        cloud_out = PointCloud2()

        fields = [pc2.PointField() for _ in range(3)]

        fields[0].name = "x"
        fields[0].offset = 0
        fields[0].datatype = pc2.PointField.FLOAT32
        fields[0].count = 1

        fields[1].name = "y"
        fields[1].offset = 4
        fields[1].datatype = pc2.PointField.FLOAT32
        fields[1].count = 1

        fields[2].name = "z"
        fields[2].offset = 8
        fields[2].datatype = pc2.PointField.FLOAT32
        fields[2].count = 1

        idx_intensity = idx_index = idx_distance = idx_timestamp = -1
        idx_vpx = idx_vpy = idx_vpz = -1

        offset = 12

        if (channel_options & self.ChannelOption.INTENSITY and
                len(scan_in.intensities) > 0):
            field_size = len(fields)
            fields.append(pc2.PointField())
            fields[field_size].name = "intensity"
            fields[field_size].datatype = pc2.PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_intensity = field_size

        if channel_options & self.ChannelOption.INDEX:
            field_size = len(fields)
            fields.append(pc2.PointField())
            fields[field_size].name = "index"
            fields[field_size].datatype = pc2.PointField.INT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_index = field_size

        if channel_options & self.ChannelOption.DISTANCE:
            field_size = len(fields)
            fields.append(pc2.PointField())
            fields[field_size].name = "distances"
            fields[field_size].datatype = pc2.PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_distance = field_size

        if channel_options & self.ChannelOption.TIMESTAMP:
            field_size = len(fields)
            fields.append(pc2.PointField())
            fields[field_size].name = "stamps"
            fields[field_size].datatype = pc2.PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_timestamp = field_size

        if channel_options & self.ChannelOption.VIEWPOINT:
            field_size = len(fields)
            fields.extend([pc2.PointField() for _ in range(3)])
            fields[field_size].name = "vp_x"
            fields[field_size].datatype = pc2.PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpx = field_size
            field_size += 1

            fields[field_size].name = "vp_y"
            fields[field_size].datatype = pc2.PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpy = field_size
            field_size += 1

            fields[field_size].name = "vp_z"
            fields[field_size].datatype = pc2.PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpz = field_size

        if range_cutoff < 0:
            range_cutoff = scan_in.range_max
        else:
            range_cutoff = min(range_cutoff, scan_in.range_max)

        points = []
        big_str = "\n"
        for i in range(N):
            ri = scan_in.ranges[i]
            if ri < range_cutoff and ri >= scan_in.range_min:
                point = output[:, i].tolist()
                point.append(0)
                p = point
                angle_increment = scan_in.angle_increment
                min_angle = scan_in.angle_min
                dist = ri
                idx = i

                if idx_intensity != -1:
                    point.append(scan_in.intensities[i])

                if idx_index != -1:
                    point.append(i)

                if idx_distance != -1:
                    point.append(scan_in.ranges[i])

                if idx_timestamp != -1:
                    point.append(i * scan_in.time_increment)

                if idx_vpx != -1 and idx_vpy != -1 and idx_vpz != -1:
                    point.extend([0 for _ in range(3)])

                points.append(point)

                big_str += ("   " + str(idx).zfill(2) + ": x: " +
                            str(round(p[0], 2)) + ", y: " + str(round(p[1], 2)) +
                            ", z: " + str(round(p[2], 2)) + " = " +
                            str(round(dist, 2)) + "m (at " +
                            str(round(degrees(idx * angle_increment + min_angle), 2)) + "deg)\n")

        cloud_out = pc2.create_cloud(scan_in.header, fields, points)
        return cloud_out


class LaserPublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__('laser_publisher')

        # Parameters (replacing ddynamic_reconfigure)
        self.declare_parameter('angle_increment', radians(120.0 * 2.0) / 61.0)
        self.declare_parameter('half_max_angle', 120.0)

        self.lp = LaserProjection()

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.all_laser_pub = self.create_publisher(LaserScan, '/pepper/laser_2', 1)
        self.pc_pub = self.create_publisher(PointCloud2, '/cloud', 1)
        self.pcl_pub = self.create_publisher(PointCloud2, '/cloudl', 1)
        self.pcr_pub = self.create_publisher(PointCloud2, '/cloudr', 1)
        self.pc_redone_pub = self.create_publisher(PointCloud2, '/cloud_redone', 1)
        self.pc_rereprojected_pub = self.create_publisher(PointCloud2, '/cloud_rereprojected', 1)

        # Subscribers via message_filters
        left_sub = message_filters.Subscriber(self, LaserScan, '/pepper/scan_left')
        front_sub = message_filters.Subscriber(self, LaserScan, '/pepper/scan_front')
        right_sub = message_filters.Subscriber(self, LaserScan, '/pepper/scan_right')

        self.ts = message_filters.TimeSynchronizer([left_sub, front_sub, right_sub], 10)
        self.ts.registerCallback(self.scan_cb)

        self.get_logger().info("LaserPublisher ready.")

    @property
    def angle_increment(self):
        return self.get_parameter('angle_increment').value

    @property
    def half_max_angle(self):
        return self.get_parameter('half_max_angle').value

    def scan_cb(self, left, front, right):
        self.get_logger().debug("scan_cb triggered")
        translated_points = []
        try:
            pc_left = self.lp.projectLaser(left, channel_options=0x00)
            pc_front = self.lp.projectLaser(front, channel_options=0x00)
            pc_right = self.lp.projectLaser(right, channel_options=0x00)
        except Exception as e:
            self.get_logger().error("Failed to project laser scan: " + str(e))
            return

        pc_left.header.stamp = self.get_clock().now().to_msg()
        pc_left.header.frame_id = 'SurroundingLeftLaser_frame'
        self.pcl_pub.publish(pc_left)
        self.pcr_pub.publish(pc_right)

        try:
            ts_right = self.tf_buffer.lookup_transform(
                'base_footprint', 'SurroundingRightLaser_frame',
                rclpy.time.Time())
        except Exception as e:
            self.get_logger().debug("TF right lookup failed: " + str(e))
            return
        self.get_logger().debug("Transform Right to Front: " + str(ts_right))
        transformed_cloud = do_transform_cloud(pc_right, ts_right)
        for p in read_points(transformed_cloud,
                             field_names=('x', 'y', 'z'),
                             skip_nans=False):
            translated_points.append(p)

        for i in range(8):
            translated_points.append((float('nan'), float('nan'), float('nan')))

        try:
            ts_front = self.tf_buffer.lookup_transform(
                'base_footprint', 'SurroundingFrontLaser_frame',
                rclpy.time.Time())
        except Exception as e:
            self.get_logger().debug("TF front lookup failed: " + str(e))
            return
        self.get_logger().debug("Transform Front to Front: " + str(ts_front))
        transformed_cloud_f = do_transform_cloud(pc_front, ts_front)

        for p in read_points(transformed_cloud_f,
                             field_names=('x', 'y', 'z'),
                             skip_nans=False):
            translated_points.append(p)

        try:
            ts_left = self.tf_buffer.lookup_transform(
                'base_footprint', 'SurroundingLeftLaser_frame',
                rclpy.time.Time())
        except Exception as e:
            self.get_logger().debug("TF left lookup failed: " + str(e))
            return
        self.get_logger().debug("Transform Left to Front: " + str(ts_left))
        transformed_cloud_l = do_transform_cloud(deepcopy(pc_left), ts_left)

        for i in range(8):
            translated_points.append((float('nan'), float('nan'), float('nan')))

        for p in read_points(transformed_cloud_l,
                             field_names=('x', 'y', 'z'),
                             skip_nans=False):
            translated_points.append(p)

        # Create a point cloud from the combined points wrt base_footprint
        pc_front.header.frame_id = 'base_footprint'
        point_cloud = create_cloud_xyz32(pc_front.header, translated_points)
        self.pc_pub.publish(point_cloud)
        self.get_logger().debug("pointcloud all together len: " +
                                str(point_cloud.width))

        # Convert combined point cloud into LaserScan
        all_laser_msg = LaserScan()
        laser_ranges, angle_min, angle_max, angle_increment = self.pc_to_laser(
            point_cloud)
        all_laser_msg.header.frame_id = 'base_footprint'
        all_laser_msg.header.stamp = self.get_clock().now().to_msg()
        all_laser_msg.ranges = laser_ranges
        all_laser_msg.angle_min = angle_min
        all_laser_msg.angle_max = angle_max
        all_laser_msg.angle_increment = angle_increment
        all_laser_msg.range_min = 0.1
        all_laser_msg.range_max = 7.0
        all_laser_msg.intensities = []
        self.all_laser_pub.publish(all_laser_msg)

        self.get_logger().debug("all_laser_msg len: " + str(len(all_laser_msg.ranges)))
        pc_redone = self.lp.projectLaser(all_laser_msg, channel_options=0x00)
        self.get_logger().debug("all_laser pc_redone len: " + str(pc_redone.width))
        self.pc_redone_pub.publish(pc_redone)

        self.get_logger().debug("point_cloud frame_id, pc_redone frame_id: " +
                                str((point_cloud.header.frame_id,
                                     pc_redone.header.frame_id)))
        compare_str = "\n"
        for idx, (point_in, point_out) in enumerate(
                zip(read_points(point_cloud), read_points(pc_redone))):
            point_out = [point_out[0], point_out[1], 0.0]
            point_in = [point_in[0], point_in[1], 0.0]
            compare_str += str(idx).zfill(2) + ":\n"
            compare_str += "  in : " + str(point_in)
            compare_str += "\n  out: " + str(point_out) + "\n"
            dist = np.linalg.norm(np.array(point_out) - np.array(point_in))
            compare_str += " dist: " + str(dist) + "\n"
            angle1 = atan2(point_in[1], point_in[0])
            angle2 = atan2(point_out[1], point_out[0])
            angle_dif = angle2 - angle1
            compare_str += " angle dif: " + str(angle_dif) + "\n"

        self.get_logger().debug(compare_str)

    def pc_to_laser(self, cloud):
        laser_points = []
        points_rereprojected = []
        multiply_num_rays = 8
        num_rays = 61 * multiply_num_rays
        laser_points2 = [float('nan')] * num_rays
        min_angle = -radians(self.half_max_angle)
        max_angle = radians(self.half_max_angle)
        angle_increment = (radians(self.half_max_angle) * 2.0) / float(num_rays)
        big_str = "\n"
        for idx, p in enumerate(read_points(cloud, skip_nans=False)):
            p = [p[0], p[1], 0.0]
            dist = np.linalg.norm(np.array((0., 0., 0.)) - np.array(p))
            big_str += ("   " + str(idx).zfill(2) + ": x: " + str(round(p[0], 2)) +
                        ", y: " + str(round(p[1], 2)) + ", z: " + str(round(p[2], 2)) +
                        " = " + str(round(dist, 2)) + "m (at " +
                        str(round(degrees(idx * angle_increment + min_angle), 2)) + "deg)\n")

            laser_points.append(dist)
            x = dist * cos(idx * angle_increment + min_angle)
            y = dist * sin(idx * angle_increment + min_angle)
            self.get_logger().debug(" [ px, py, are the correct points ] ")
            if dist is None:
                self.get_logger().warning("dist is None, setting to 0.0")
                dist = 0.0
            else:
                self.get_logger().debug(f"dist, px, py: {dist} {p[0]} {p[1]}")
                self.get_logger().debug(f"dist, x, y:   {dist} {x} {y}")
            dist_from_rereproj = self.get_dist(x, y)
            self.get_logger().debug("dist rereproj: " + str(dist_from_rereproj))

            points_rereprojected.append((x, y, 0.0))

            angle = atan2(p[1], p[0])
            expected_angle = idx * self.angle_increment + min_angle
            if not isnan(angle):
                tmp_angle = angle - min_angle
                self.get_logger().debug(f"tmp_angle: {degrees(tmp_angle)} deg")
                self.get_logger().debug("angle_increment: " + str(degrees(angle_increment)))
                closest_index = int(tmp_angle / angle_increment)
                self.get_logger().debug("closest index: " + str(closest_index))
                # Discard points outside the scan cone — do NOT clamp to boundary
                # slots, as that would place e.g. a +170° reading at the -120°
                # position and corrupt navigation costmap data.
                if 0 <= closest_index < len(laser_points2):
                    laser_points2[closest_index] = dist
            else:
                self.get_logger().debug("nan, not adding anything to scan")

            self.get_logger().debug("Angle from p : " + str(round(degrees(angle), 2)))
            self.get_logger().debug("Expected angle: " + str(round(degrees(expected_angle), 2)))

        self.get_logger().debug("Lasered cloud")
        self.get_logger().debug(big_str)

        laser_points = laser_points2
        self.get_logger().debug("Len of laser points after new technique: " +
                                str(len(laser_points)))

        rereprojected_pc = PointCloud2()
        rereprojected_pc.header.frame_id = 'base_footprint'
        rereprojected_pc.header.stamp = self.get_clock().now().to_msg()
        point_cloud_rere = create_cloud_xyz32(
            rereprojected_pc.header, points_rereprojected)
        self.pc_rereprojected_pub.publish(point_cloud_rere)

        return laser_points, min_angle, max_angle, angle_increment

    def get_dist(self, x0, y0, x1=0.0, y1=0.0):
        return sqrt((x1 - x0)**2 + (y1 - y0)**2)


if __name__ == "__main__":
    rclpy.init()
    try:
        lp = LaserPublisher()
        rclpy.spin(lp)
    except KeyboardInterrupt:
        pass
    finally:
        lp.destroy_node()
        rclpy.shutdown()
