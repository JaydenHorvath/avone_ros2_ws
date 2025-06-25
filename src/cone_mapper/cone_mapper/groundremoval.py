#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import numpy as np
import pcl

class GroundRemovalRANSACNode(Node):
    def __init__(self):
        super().__init__('ground_removal_ransac_node')

        # ——— RANSAC parameters —————————————————————————————————
        self.declare_parameter('dist_thresh', 0.02)
        self.declare_parameter('max_iter',   100)
        self.declare_parameter('downsample_leaf', 0.02)  # 2 cm voxels

        self.dist_thresh      = self.get_parameter('dist_thresh').get_parameter_value().double_value
        self.max_iter         = self.get_parameter('max_iter').get_parameter_value().integer_value
        self.downsample_leaf  = self.get_parameter('downsample_leaf').get_parameter_value().double_value

        # Initialize last_publish so throttle check won’t crash:
        self.last_publish = self.get_clock().now()

        # Subscribe with QoS=BEST_EFFORT, depth=1 to avoid buffering old clouds:
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=   QoSHistoryPolicy.KEEP_LAST,
            depth=     1
        )

        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',  # ← adjust to your actual depth‐cloud topic
            self.cloud_callback,
            qos
        )

        # self.pc_sub = self.create_subscription(
        #     PointCloud2,
        #     '/camera/rgbd/points',  # ← adjust to your actual depth‐cloud topic
        #     self.cloud_callback,
        #     qos
        # )

        # Publisher for the “no‐ground” cloud:
        self.pc_pub = self.create_publisher(
            PointCloud2,
            '/cloud_no_ground_ransac',
            10  # queue size for publishing
        )

        self.get_logger().info("GroundRemovalRANSACNode initialized. Throttling to 10 Hz, downsample_leaf=%.3f m" % self.downsample_leaf)

    def cloud_callback(self, msg: PointCloud2):
        # 0) Throttle: process at most 10 Hz
        now = self.get_clock().now()
        if (now - self.last_publish).nanoseconds < int(1e9 / 10):
            return
        self.last_publish = now

        # 1) Read (x,y,z) tuples from PointCloud2, skipping NaNs
        xyz_list = [
            (pt[0], pt[1], pt[2])
            for pt in point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            )
        ]
        if not xyz_list:
            self.get_logger().warn('Received empty or all‐NaN PointCloud2')
            return

        # 2) Convert to an Nx3 float32 NumPy array
        cloud_arr = np.asarray(xyz_list, dtype=np.float32)

        # ——— Crop out the car’s front via a fixed bounding box —————————————————
        xmin, xmax =  0.0, 2.0    # forward
        ymin, ymax = -0.8, 0.8    # left/right
        zmin, zmax = -1.0, 0.0    # up/down

        mask = ~(
            (cloud_arr[:, 0] > xmin) & (cloud_arr[:, 0] < xmax) &
            (cloud_arr[:, 1] > ymin) & (cloud_arr[:, 1] < ymax) &
            (cloud_arr[:, 2] > zmin) & (cloud_arr[:, 2] < zmax)
        )
        cloud_arr = cloud_arr[mask]
        # —————————————————————————————————————————————————————————————————————————

        # 3) Build a PCL PointCloud from the cropped array
        pcl_cloud = pcl.PointCloud(cloud_arr)

        # 3a) Downsample with VoxelGrid
        try:
            vg = pcl_cloud.make_voxel_grid_filter()
            vg.set_leaf_size(self.downsample_leaf,
                             self.downsample_leaf,
                             self.downsample_leaf)
            pcl_cloud = vg.filter()
        except Exception as e:
            self.get_logger().warn(f"VoxelGrid downsampling failed: {e}")
            # If downsample fails, just proceed with the full cropped cloud

        # 4) Create the segmenter & configure RANSAC
        seg = pcl_cloud.make_segmenter()
        try:
            seg.set_model_type(pcl.SACMODEL_PLANE)
        except AttributeError:
            seg.set_ModelType(pcl.SACMODEL_PLANE)

        try:
            seg.set_method_type(pcl.SAC_RANSAC)
        except AttributeError:
            seg.set_MethodType(pcl.SAC_RANSAC)

        try:
            seg.set_distance_threshold(self.dist_thresh)
        except AttributeError:
            seg.set_DistanceThreshold(self.dist_thresh)

        try:
            seg.set_max_iterations(self.max_iter)
        except AttributeError:
            seg.set_MaxIterations(self.max_iter)

        # 5) Run segmentation
        try:
            inlier_indices, _ = seg.segment()
        except Exception as e:
            self.get_logger().error(f"RANSAC segmentation failed: {e}")
            # If segmentation failed, re‐publish the cropped+downsampled cloud
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = msg.header.frame_id
            pts = np.asarray(pcl_cloud, dtype=np.float32).tolist()
            cloud_out = point_cloud2.create_cloud_xyz32(header, pts)
            self.pc_pub.publish(cloud_out)
            return

        # 6) If no plane was found, re‐publish the cropped+downsampled cloud
        if not inlier_indices:
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = msg.header.frame_id
            pts = np.asarray(pcl_cloud, dtype=np.float32).tolist()
            cloud_out = point_cloud2.create_cloud_xyz32(header, pts)
            self.pc_pub.publish(cloud_out)
            return

        # 7) Extract everything except the plane (negative=True removes the ground)
        cloud_filtered_pcl = pcl_cloud.extract(inlier_indices, negative=True)

        # Convert the filtered PCL cloud back to a list of (x,y,z)
        filtered_arr   = np.asarray(cloud_filtered_pcl, dtype=np.float32)
        filtered_pts   = filtered_arr.tolist()

        # 8) Publish the filtered “no‐ground” cloud
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id
        new_cloud = point_cloud2.create_cloud_xyz32(header, filtered_pts)
        self.pc_pub.publish(new_cloud)


def main(args=None):
    rclpy.init(args=args)

    node = GroundRemovalRANSACNode()
    # Use a MultiThreadedExecutor to avoid blocking on slow callbacks:
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
