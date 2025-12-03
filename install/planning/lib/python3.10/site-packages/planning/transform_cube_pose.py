# import rclpy
# from rclpy.node import Node
# from tf2_ros import Buffer, TransformListener
# from geometry_msgs.msg import PointStamped 
# import tf2_geometry_msgs
# from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
# from ros2_aruco_interfaces.msg import ArucoMarkers

# class TransformCubePose(Node):
#     def __init__(self):
#         super().__init__('transform_cube_pose')

#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         # self.cube_pose_sub = self.create_subscription(
#         #     PointStamped,
#         #     '/cube_pose',
#         #     self.cube_pose_callback,
#         #     10
#         # )

#         self.aruco_sub = self.create_subscription(
#             ArucoMarkers, # Use the imported ArucoMarkers type
#             '/aruco_markers', # Use the confirmed topic name
#             self.aruco_markers_callback,
#             10
#         )

#         self.cube_pose_pub = self.create_publisher(PointStamped, '/cube_pose_in_base', 1)

#         self.object_marker_id = 1

#         rclpy.spin_once(self, timeout_sec=2)
#         self.cube_pose = None

#     # def cube_pose_callback(self, msg: PointStamped):
#     #     # if self.cube_pose is None:
#     #     self.cube_pose = self.transform_cube_pose(msg)
#     #     self.cube_pose_pub.publish(self.cube_pose)

#     def aruco_markers_callback(self, msg: ArucoMarkers):
#             """
#             Callback to process ArucoMarkers message, find the position of Marker ID 1,
#             and transform it from the camera frame to the 'base_link' frame.
#             """
            
#             marker_pose_in_camera = None

#             # 1. Iterate to find the Pose of the target Marker (ID 1)
#             for i, marker_id in enumerate(msg.marker_ids):
#                 if marker_id == self.object_marker_id: # ID 1
#                     # The 'poses' field is a list of geometry_msgs/Pose
#                     marker_pose_in_camera = msg.poses[i] 
#                     break

#             if marker_pose_in_camera is None:
#                 # self.get_logger().debug(f"Marker ID {self.object_marker_id} not found.")
#                 return

#             # 2. Convert the Pose (which contains position) to a PointStamped
#             point_stamped_in_camera = PointStamped()
#             point_stamped_in_camera.header = msg.header
#             # The position (x, y, z) is the tvec from the Aruco node log
#             point_stamped_in_camera.point = marker_pose_in_camera.position 
#             self.cube_pose = self.transform_cube_pose(point_stamped_in_camera)
#             self.cube_pose_pub.publish(self.cube_pose)

#     def transform_cube_pose(self, msg: PointStamped):
#         """ 
#         Transform point into base_link frame
#         Args: 
#             - msg: PointStamped - The message from /cube_pose, of the position of the cube in camera_depth_optical_frame
#         Returns:
#             Point: point in base_link_frame in form [x, y, z]
#         """
#         msg.header.stamp = rclpy.time.Time().to_msg()        
#         # ts1 = self.tf_buffer.lookuptransform('ar', "camera_depth_optical_frame")

#         tp1 = self.tf_buffer.transform(msg, 'ar_marker_10', timeout=rclpy.duration.Duration(seconds=3.0))
#         tp2 = self.tf_buffer.transform(tp1, 'base_link', timeout=rclpy.duration.Duration(seconds=3.0))

#         cube_pose = PointStamped()
#         # Fill in message
#         cube_pose.header.stamp = rclpy.time.Time().to_msg()  
#         cube_pose.header.frame_id = "base_link"
#         cube_pose.point = tp2.point

#         return cube_pose


# def main(args=None):
#     rclpy.init(args=args)
#     node = TransformCubePose()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from tf2_ros import Buffer, TransformListener
# from geometry_msgs.msg import PoseStamped # CHANGED: Import PoseStamped
# import tf2_geometry_msgs # Required for transforming Poses
# from ros2_aruco_interfaces.msg import ArucoMarkers

# class TransformCubePose(Node):
#     def __init__(self):
#         super().__init__('transform_cube_pose')

#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         self.aruco_sub = self.create_subscription(
#             ArucoMarkers,
#             '/aruco_markers',
#             self.aruco_markers_callback,
#             10
#         )

#         # CHANGED: Publisher now publishes PoseStamped
#         self.cube_pose_pub = self.create_publisher(PoseStamped, '/cube_pose_in_base', 1)

#         self.object_marker_id = 1
#         self.cube_pose = None

#     def aruco_markers_callback(self, msg: ArucoMarkers):
#         """
#         Callback to process ArucoMarkers message, find the Pose of Marker ID 1,
#         and transform it from the camera frame to the 'base_link' frame.
#         """
#         marker_pose_in_camera = None

#         # 1. Iterate to find the Pose of the target Marker (ID 1)
#         for i, marker_id in enumerate(msg.marker_ids):
#             if marker_id == self.object_marker_id:
#                 marker_pose_in_camera = msg.poses[i] 
#                 break

#         if marker_pose_in_camera is None:
#             return

#         # 2. Create PoseStamped (Preserve Orientation!)
#         pose_stamped_in_camera = PoseStamped()
#         pose_stamped_in_camera.header = msg.header # CRITICAL: Keep original timestamp
#         pose_stamped_in_camera.pose = marker_pose_in_camera # CHANGED: Assign full pose

#         # 3. Transform
#         try:
#             self.cube_pose = self.transform_cube_pose(pose_stamped_in_camera)
#             if self.cube_pose:
#                 self.cube_pose_pub.publish(self.cube_pose)
#         except Exception as e:
#             self.get_logger().warn(f"Transform failed: {e}")

#     def transform_cube_pose(self, input_pose: PoseStamped):
#         """ 
#         Transform Pose (Position + Orientation) into base_link frame
#         """
#         # CHANGED: We do not overwrite the timestamp here. We use the one from the camera.
        
#         # We ask TF to transform directly to 'base_link'.
#         # TF2 will find the path: Camera -> Ar_Marker_10 -> Base_Link automatically.
#         try:
#             # timeout allows the buffer to wait slightly if the transform is delayed
#             pose_in_base = self.tf_buffer.transform(
#                 input_pose, 
#                 'base_link', 
#                 timeout=rclpy.duration.Duration(seconds=0.1)
#             )
#             return pose_in_base
#         except (LookupException, ConnectivityException, ExtrapolationException) as e:
#             self.get_logger().warn(f"TF Error: {e}")
#             return None

# def main(args=None):
#     rclpy.init(args=args)
#     node = TransformCubePose()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
# Import all necessary TF2 exceptions for robust code
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs # Required for transforming Poses
from ros2_aruco_interfaces.msg import ArucoMarkers

class TransformTwoArucoPoses(Node):
    def __init__(self):
        super().__init__('transform_two_aruco_poses')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.aruco_sub = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.aruco_markers_callback,
            10
        )

        # Publisher for the CUBE's pickup pose
        self.cube_pose_pub = self.create_publisher(PoseStamped, '/cube_pose_in_base', 1)
        
        # NEW: Publisher for the DROP-OFF pose
        self.drop_off_pose_pub = self.create_publisher(PoseStamped, '/drop_off_pose_in_base', 1)

        # Marker IDs
        self.cube_marker_id = 1      # ID for the object (Cube) to be picked up
        self.drop_off_marker_id = 2  # NEW ID for the drop-off location

    def aruco_markers_callback(self, msg: ArucoMarkers):
        """
        Callback to process ArucoMarkers message, find the Pose of two markers,
        and transform them from the camera frame to the 'base_link' frame.
        """
        marker_poses_in_camera = {}
        
        # 1. Iterate to find the Poses of the target Markers (ID 1 and ID 2)
        for i, marker_id in enumerate(msg.marker_ids):
            if marker_id == self.cube_marker_id:
                # Store the PoseStamped for the cube
                pose_stamped_in_camera = PoseStamped()
                pose_stamped_in_camera.header = msg.header
                pose_stamped_in_camera.pose = msg.poses[i]
                marker_poses_in_camera['cube'] = pose_stamped_in_camera

            elif marker_id == self.drop_off_marker_id:
                # Store the PoseStamped for the drop-off location
                pose_stamped_in_camera = PoseStamped()
                pose_stamped_in_camera.header = msg.header
                pose_stamped_in_camera.pose = msg.poses[i]
                marker_poses_in_camera['drop_off'] = pose_stamped_in_camera

        # 2. Transform and Publish Cube Pose
        if 'cube' in marker_poses_in_camera:
            try:
                cube_pose_in_base = self.transform_pose(marker_poses_in_camera['cube'])
                if cube_pose_in_base:
                    self.cube_pose_pub.publish(cube_pose_in_base)
            except Exception as e:
                self.get_logger().warn(f"Cube Transform failed: {e}")

        # 3. Transform and Publish Drop-Off Pose
        if 'drop_off' in marker_poses_in_camera:
            try:
                drop_off_pose_in_base = self.transform_pose(marker_poses_in_camera['drop_off'])
                if drop_off_pose_in_base:
                    self.drop_off_pose_pub.publish(drop_off_pose_in_base)
            except Exception as e:
                self.get_logger().warn(f"Drop-Off Transform failed: {e}")

    def transform_pose(self, input_pose: PoseStamped):
        """ Transform Pose (Position + Orientation) into base_link frame """
        try:
            # timeout allows the buffer to wait slightly if the transform is delayed
            pose_in_base = self.tf_buffer.transform(
                input_pose, 
                'base_link', 
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return pose_in_base
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF Error: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = TransformTwoArucoPoses() # CHANGED: Class name
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()