import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped 
import tf2_geometry_msgs
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class TransformCubePose(Node):
    def __init__(self):
        super().__init__('transform_cube_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cube_pose_sub = self.create_subscription(
            PointStamped,
            '/cube_pose',
            self.cube_pose_callback,
            10
        )

        self.cube_pose_pub = self.create_publisher(PointStamped, '/cube_pose_in_base', 1)

        rclpy.spin_once(self, timeout_sec=2)
        self.cube_pose = None

    def cube_pose_callback(self, msg: PointStamped):
        # if self.cube_pose is None:
        self.cube_pose = self.transform_cube_pose(msg)
        self.cube_pose_pub.publish(self.cube_pose)

    def transform_cube_pose(self, msg: PointStamped):
        """ 
        Transform point into base_link frame
        Args: 
            - msg: PointStamped - The message from /cube_pose, of the position of the cube in camera_depth_optical_frame
        Returns:
            Point: point in base_link_frame in form [x, y, z]
        """
        msg.header.stamp = rclpy.time.Time().to_msg()        
        # ts1 = self.tf_buffer.lookuptransform('ar', "camera_depth_optical_frame")

        tp1 = self.tf_buffer.transform(msg, 'ar_marker_6', timeout=rclpy.duration.Duration(seconds=3.0))
        tp2 = self.tf_buffer.transform(tp1, 'base_link', timeout=rclpy.duration.Duration(seconds=3.0))

        cube_pose = PointStamped()
        # Fill in message
        cube_pose.header.stamp = rclpy.time.Time().to_msg()  
        cube_pose.header.frame_id = "base_link"
        cube_pose.point = tp2.point

        return cube_pose


def main(args=None):
    rclpy.init(args=args)
    node = TransformCubePose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
