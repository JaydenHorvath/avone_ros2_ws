#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np

# Make sure python-pcl is installed (e.g. `sudo apt install python3-pcl`)
import pcl


class GroundRemovalRANSACNode(Node):
    def __init__(self):
        super().__init__('ground_removal_ransac_node')

        # RANSAC parameters (parameters can be overridden via --ros-args -p ...)
        self.declare_parameter('dist_thresh', 0.02)
        self.declare_parameter('max_iter', 100)

        self.dist_thresh = self.get_parameter('dist_thresh').get_parameter_value().double_value
        self.max_iter = self.get_parameter('max_iter').get_parameter_value().integer_value

        # Subscribe to your depth‐camera point cloud:
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/lidar/points',  # ← adjust this topic to whatever your camera publishes
            self.cloud_callback,
            10
        )

        # Publish the “no‐ground” cloud here:
        self.pc_pub = self.create_publisher(
            PointCloud2,
            '/lidar_no_ground_ransac',
            10
        )

    def cloud_callback(self, msg: PointCloud2):
        # 1) Read (x,y,z) tuples out of PointCloud2
        xyz_list = [
            (pt[0], pt[1], pt[2])
            for pt in point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            )
        ]
        if not xyz_list:
            self.get_logger().warn('Received an empty or all‐NaN PointCloud2')
            return

        # 2) Convert to an Nx3 float32 numpy array
        cloud_arr = np.asarray(xyz_list, dtype=np.float32)

        # ──────────── Limit range to 20 meters ────────────
        dists = np.linalg.norm(cloud_arr, axis=1)
        range_mask = dists <= 20.0
        cloud_arr = cloud_arr[range_mask]
        # ────────────────────────────────────────────────────

        # 3) Build a PCL PointCloud from the filtered array
        pcl_cloud = pcl.PointCloud(cloud_arr)

        # 4) Create the segmenter and configure RANSAC (try both naming styles)
        seg = pcl_cloud.make_segmenter()

        # model type (PLANE)
        try:
            seg.set_model_type(pcl.SACMODEL_PLANE)
        except AttributeError:
            seg.set_ModelType(pcl.SACMODEL_PLANE)

        # method type (RANSAC)
        try:
            seg.set_method_type(pcl.SAC_RANSAC)
        except AttributeError:
            seg.set_MethodType(pcl.SAC_RANSAC)

        # distance threshold
        try:
            seg.set_distance_threshold(self.dist_thresh)
        except AttributeError:
            seg.set_DistanceThreshold(self.dist_thresh)

        # max iterations
        try:
            seg.set_max_iterations(self.max_iter)
        except AttributeError:
            seg.set_MaxIterations(self.max_iter)

        # 5) Run segmentation
        try:
            inlier_indices, _ = seg.segment()
        except Exception as e:
            self.get_logger().error(f'RANSAC segmentation failed: {e}')
            # On failure, republish the full (range‐limited) cloud so downstream nodes still see something:
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = msg.header.frame_id
            full_cloud = point_cloud2.create_cloud_xyz32(header, cloud_arr.tolist())
            self.pc_pub.publish(full_cloud)
            return

        # 6) If no inliers (no plane found), republish full (range‐limited) cloud
        if not inlier_indices:
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = msg.header.frame_id
            full_cloud = point_cloud2.create_cloud_xyz32(header, cloud_arr.tolist())
            self.pc_pub.publish(full_cloud)
            return

        # 7) Extract everything except the plane (negative=True removes the ground)
        cloud_filtered_pcl = pcl_cloud.extract(inlier_indices, negative=True)

        # Convert the resulting PCL cloud back to a list of (x,y,z) for PointCloud2
        filtered_arr = np.asarray(cloud_filtered_pcl, dtype=np.float32)
        filtered_points = filtered_arr.tolist()

        # 8) Publish the filtered cloud
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id
        new_cloud = point_cloud2.create_cloud_xyz32(header, filtered_points)
        self.pc_pub.publish(new_cloud)
        # end of cloud_callback


def main(args=None):
    rclpy.init(args=args)
    node = GroundRemovalRANSACNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
