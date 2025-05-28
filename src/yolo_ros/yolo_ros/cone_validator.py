import rclpy
import math
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs_py import point_cloud2
import tf2_geometry_msgs  # <--- registers the PoseStamped → PoseStamped converter


class ConeValidator(Node):
    def __init__(self):
        super().__init__('cone_validator')

                # Publisher for only the validated cones
                # Publisher for only the validated cones
        self.valid_pub = self.create_publisher(
            PoseStamped, '/validated_cone', 10)

        # TF2 setup
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # subs & pubs
        self.pc_sub = self.create_subscription(
            PointCloud2, '/lidar/points', self.pc_cb, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/cone_pose', self.pose_cb, 10)

        self.last_cloud = None

    def pc_cb(self, msg: PointCloud2):
        self.last_cloud = msg

    def pose_cb(self, ps: PoseStamped):
        if self.last_cloud is None:
            return

        # 1) transform the PoseStamped into LiDAR frame in one go
        try:
            # rclpy.duration.Duration is fine here for timeout
            ps_lidar: PoseStamped = self.tf_buffer.transform(
                ps,
                self.last_cloud.header.frame_id,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except TransformException as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return


        # 2) extract the XYZ center
        cx = ps_lidar.pose.position.x
        cy = ps_lidar.pose.position.y
        cz = ps_lidar.pose.position.z
        self.get_logger().info(f"Checking cone at lidar coords: x={cx:.2f}, y={cy:.2f}, z={cz:.2f}")

        
        # after transforming into lidar frame…
        r = math.sqrt(cx*cx + cy*cy + cz*cz)
        if r > 12.0:    # your sensor’s max range
            self.get_logger().warn(f"Skipping cone at {r:.1f} m—beyond LiDAR range")
            return


                # this yields (x,y,z) tuples
        gen = point_cloud2.read_points(
            self.last_cloud,
            field_names=('x','y','z'),
            skip_nans=True
        )

        import numpy as np
        pts_raw = np.array(list(gen))  # structured array with dtype.names ('x','y','z')

        # Convert to a plain (N,3) float array:
        pts = np.column_stack((pts_raw['x'], pts_raw['y'], pts_raw['z']))

        # Now you can subtract:
        center = np.array([cx, cy, cz])
        # tolerances
        R_XY = 0.5   # 0.5 m in XY
        DZ   = 0.3   # ±0.3 m in Z

        dx     = pts[:,0] - cx
        dy     = pts[:,1] - cy
        dz_pts = pts[:,2] - cz

        mask_xy = (dx*dx + dy*dy) < (R_XY*R_XY)
        mask_z  = (dz_pts > -DZ) & (dz_pts < +DZ)
        mask    = mask_xy & mask_z
        hits    = pts[mask]

        MIN_HITS = 3
        # 5) validate
        if hits.shape[0] >= MIN_HITS:
            self.get_logger().info(
                f"Cone validated at {center}, {hits.shape[0]} points")
            # republish the PoseStamped in lidar frame (or camera frame if you prefer)
            valid_ps = PoseStamped()
            valid_ps.header = ps_lidar.header
            valid_ps.pose   = ps_lidar.pose
            self.valid_pub.publish(valid_ps)
        else:
            self.get_logger().warn(
                f"False positive? only {hits.shape[0]} LiDAR points")

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = ConeValidator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()