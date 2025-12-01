#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import StaticTransformBroadcaster
import tf2_geometry_msgs

import numpy as np
import scipy.spatial.transform as st

class ConstantTransformPublisher(Node):
    def __init__(self):
        super().__init__('constant_tf_publisher')
        self.br = StaticTransformBroadcaster(self)

        self.declare_parameter('ar_marker', 'ar_marker_10')
        marker = self.get_parameter('ar_marker').get_parameter_value().string_value

        # Homogeneous transform G_ar->base_link
        G = np.array([
            [-1, 0, 0, 0.0],
            [ 0, 0, 1, 0.16],
            [ 0, 1, 0, -0.13],
            [ 0, 0, 0, 1.0]
        ])

        # Create TransformStamped
        self.transform = TransformStamped()
        # ---------------------------
        # TODO: Fill out TransformStamped message
        # --------------------------
        # Extract rotation (3x3) and translation (3x1)
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.transform.header.frame_id = 'ar_marker_10'
        self.transform.child_frame_id = 'base_link'
        self.transform.transform.translation.x = G[0, 3]
        self.transform.transform.translation.y = G[1, 3]
        self.transform.transform.translation.z = G[2, 3]

        scipy_quat = st.Rotation.from_matrix(G[0:3, 0:3])
        scipy_quat = scipy_quat.as_quat()

        ros2_quat_msg = Quaternion()

        ros2_quat_msg.x = scipy_quat[0]
        ros2_quat_msg.y = scipy_quat[1]
        ros2_quat_msg.z = scipy_quat[2]
        ros2_quat_msg.w = scipy_quat[3]

        self.transform.transform.rotation = ros2_quat_msg

        self.timer = self.create_timer(0.05, self.broadcast_tf)

    def broadcast_tf(self):
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.transform)

def main():
    rclpy.init()
    node = ConstantTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
