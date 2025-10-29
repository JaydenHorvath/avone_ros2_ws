#!/usr/bin/env python3
"""
ROS2 Node: IMU Frame Transformer
--------------------------------
Subscribes to an IMU topic in one frame (e.g. 'vn100_imu_link'),
transforms the IMU data into another frame (e.g. 'imu_link') using TF2,
and republishes it for visualization or downstream use.

Example:
    Input topic:  /imu/data  (frame_id = vn100_imu_link)
    Output topic: /imu/data_imu_link  (frame_id = imu_link)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf_transformations


class IMUFrameTransformer(Node):
    def __init__(self):
        super().__init__('imu_frame_transformer')

        # Parameters
        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('target_frame', 'imu_link')

        self.input_topic = self.get_parameter('input_topic').value
        self.target_frame = self.get_parameter('target_frame').value

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers and subscribers
        self.publisher_ = self.create_publisher(Imu, f'{self.input_topic}_{self.target_frame}', 10)
        self.subscription = self.create_subscription(Imu, self.input_topic, self.imu_callback, 10)

        self.get_logger().info(f"✅ IMU Frame Transformer started.")
        self.get_logger().info(f"Listening on: {self.input_topic}")
        self.get_logger().info(f"Target frame: {self.target_frame}")
        self.get_logger().info(f"Publishing transformed data to: {self.input_topic}_{self.target_frame}")

    def imu_callback(self, msg: Imu):
        try:
            # Lookup the latest transform from IMU frame to target frame
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )

            # === Transform orientation ===
            # Convert quaternion from msg and transform
            q_msg = [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ]
            q_tf = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]

            # Multiply quaternions: q_out = q_tf * q_msg
            q_out = tf_transformations.quaternion_multiply(q_tf, q_msg)

            # Create new message
            imu_out = Imu()
            imu_out.header.stamp = msg.header.stamp
            imu_out.header.frame_id = self.target_frame

            imu_out.orientation.x = q_out[0]
            imu_out.orientation.y = q_out[1]
            imu_out.orientation.z = q_out[2]
            imu_out.orientation.w = q_out[3]

            imu_out.orientation_covariance = msg.orientation_covariance

            # === Transform angular velocity ===
            gyro_in = Vector3Stamped()
            gyro_in.header = msg.header
            gyro_in.vector = msg.angular_velocity
            gyro_out = tf_transformations.quaternion_matrix(q_tf)[:3, :3].dot(
                [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            )
            imu_out.angular_velocity.x, imu_out.angular_velocity.y, imu_out.angular_velocity.z = gyro_out
            imu_out.angular_velocity_covariance = msg.angular_velocity_covariance

            # === Transform linear acceleration ===
            accel_out = tf_transformations.quaternion_matrix(q_tf)[:3, :3].dot(
                [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
            )
            imu_out.linear_acceleration.x, imu_out.linear_acceleration.y, imu_out.linear_acceleration.z = accel_out
            imu_out.linear_acceleration_covariance = msg.linear_acceleration_covariance

            # Publish transformed message
            self.publisher_.publish(imu_out)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn_throttle(2.0, f"TF transform unavailable: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = IMUFrameTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
