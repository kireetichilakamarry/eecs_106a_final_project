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
        self.calib_pose_pub = self.create_publisher(PoseStamped, '/calibration_pose_in_base', 1)

        # Marker IDs
        self.cube_marker_id = 1      
        self.drop_off_marker_id = 2  
        self.calib_marker_id = 3

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

            elif marker_id == self.calib_marker_id:
                pose_stamped = PoseStamped()
                pose_stamped.header = msg.header
                pose_stamped.pose = msg.poses[i]
                marker_poses_in_camera['calib'] = pose_stamped

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

        if 'calib' in marker_poses_in_camera:
            try:
                calib_pose_in_base = self.transform_pose(marker_poses_in_camera['calib'])
                if calib_pose_in_base:
                    self.calib_pose_pub.publish(calib_pose_in_base)
            except Exception as e:
                self.get_logger().warn(f"Calib Transform failed: {e}")

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