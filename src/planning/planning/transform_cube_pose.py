import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped 
import tf2_geometry_msgs
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from ros2_aruco_interfaces.msg import ArucoMarkers

class TransformCubePose(Node):
    def __init__(self):
        super().__init__('transform_cube_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # self.cube_pose_sub = self.create_subscription(
        #     PointStamped,
        #     '/cube_pose',
        #     self.cube_pose_callback,
        #     10
        # )

        self.aruco_sub = self.create_subscription(
            ArucoMarkers, # Use the imported ArucoMarkers type
            '/aruco_markers', # Use the confirmed topic name
            self.aruco_markers_callback,
            10
        )

        self.cube_pose_pub = self.create_publisher(PointStamped, '/cube_pose_in_base', 1)

        self.object_marker_id = 1

        rclpy.spin_once(self, timeout_sec=2)
        self.cube_pose = None

    # def cube_pose_callback(self, msg: PointStamped):
    #     # if self.cube_pose is None:
    #     self.cube_pose = self.transform_cube_pose(msg)
    #     self.cube_pose_pub.publish(self.cube_pose)

    def aruco_markers_callback(self, msg: ArucoMarkers):
            """
            Callback to process ArucoMarkers message, find the position of Marker ID 1,
            and transform it from the camera frame to the 'base_link' frame.
            """
            
            marker_pose_in_camera = None

            # 1. Iterate to find the Pose of the target Marker (ID 1)
            for i, marker_id in enumerate(msg.marker_ids):
                if marker_id == self.object_marker_id: # ID 1
                    # The 'poses' field is a list of geometry_msgs/Pose
                    marker_pose_in_camera = msg.poses[i] 
                    break

            if marker_pose_in_camera is None:
                # self.get_logger().debug(f"Marker ID {self.object_marker_id} not found.")
                return

            # 2. Convert the Pose (which contains position) to a PointStamped
            point_stamped_in_camera = PointStamped()
            point_stamped_in_camera.header = msg.header
            # The position (x, y, z) is the tvec from the Aruco node log
            point_stamped_in_camera.point = marker_pose_in_camera.position 
            self.cube_pose = self.transform_cube_pose(point_stamped_in_camera)
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

        tp1 = self.tf_buffer.transform(msg, 'ar_marker_10', timeout=rclpy.duration.Duration(seconds=3.0))
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
