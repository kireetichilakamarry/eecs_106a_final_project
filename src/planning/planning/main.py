# # ROS Libraries
# from std_srvs.srv import Trigger
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from control_msgs.action import FollowJointTrajectory
# from geometry_msgs.msg import PointStamped 
# from moveit_msgs.msg import RobotTrajectory
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from sensor_msgs.msg import JointState
# from tf2_ros import Buffer, TransformListener
# from scipy.spatial.transform import Rotation as R
# import numpy as np
# import time

# from planning.ik import IKPlanner

# class UR7e_CubeGrasp(Node):
#     def __init__(self):
#         super().__init__('cube_grasp')

#         self.cube_pub = self.create_subscription(PointStamped, '/cube_pose_in_base', self.cube_callback, 1) # TODO: CHECK IF TOPIC ALIGNS WITH YOURS
#         self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)

#         self.exec_ac = ActionClient(
#             self, FollowJointTrajectory,
#             '/scaled_joint_trajectory_controller/follow_joint_trajectory'
#         )

#         self.gripper_cli = self.create_client(Trigger, '/toggle_gripper')

#         self.cube_pose = None
#         self.current_plan = None
#         self.joint_state = None

#         self.ik_planner = IKPlanner()

#         self.job_queue = [] # Entries should be of type either JointState or String('toggle_grip')

#     def joint_state_callback(self, msg: JointState):
#         self.joint_state = msg

#     def cube_callback(self, cube_pose):
#         if self.cube_pose is not None:
#             return

#         if self.joint_state is None:
#             self.get_logger().info("No joint state yet, cannot proceed")
#             return

#         print("cube callback:" , cube_pose)
#         self.cube_pose = cube_pose


#         # -----------------------------------------------------------
#         # Code called to Pick up pencil from holder and navigate to the IPad and begin scrolling motion
#         # Should work with Aruco Tags instead of the current vision cloud.
#         # -----------------------------------------------------------

#         # PART 1, PICK UP

#         # 1) Move to Pre-Grasp Position (gripper above the pencil)
#         '''
#         Default values from lab 
#         Use the following offsets for pre-grasp position:
#         x offset: 0.0
#         y offset: -0.035 
#         z offset: +0.185 (to be above the cube by accounting for gripper length)
#         '''
#         #joint_state = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + 0.0, self.cube_pose.point.y - 0.035, self.cube_pose.point.z + 0.35)
#         joint_state = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + 0.0, self.cube_pose.point.y, self.cube_pose.point.z + 0.35)
#         self.job_queue.append(joint_state)

#         # 2) Move to Grasp Position (lower the gripper to the pencil)
#         '''
#         Note that this will again be defined relative to the cube pose. 
#         DO NOT CHANGE z offset lower than +0.16. 
#         '''

#         """
#         joint_state = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + 0.0, self.cube_pose.point.y - 0.035, self.cube_pose.point.z + 0.21)

#         self.job_queue.append(joint_state)
        


#         # 3) Close the gripper.
#         self.job_queue.append("toggle_grip")

#         # 4) Pick up pencil from the holder
#         joint_state = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + 0.0, self.cube_pose.point.y - 0.035, self.cube_pose.point.z + 0.4)

#         self.job_queue.append(joint_state)
        
#         # 5) Navigate to the ipad
#         #TODO: UTilize Ipad Aurco Tags
#         joint_state = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + 0.25, self.cube_pose.point.y - 0.035, self.cube_pose.point.z + 0.24)
#         self.job_queue.append(joint_state)

#         # PART 2, IPAD


#         # 1) Scroling Motion
#         #TODO: Make a function to scroll continiously or for x iterations
        
        
#         joint_state = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + 0.25, self.cube_pose.point.y - 0.035, self.cube_pose.point.z + 0.239)
#         self.job_queue.append(joint_state)

#         joint_state = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + 0.25, self.cube_pose.point.y, self.cube_pose.point.z + 0.239)
#         self.job_queue.append(joint_state)
        

#         # 2) Move to Share button
#         #Probbaly code a prexisting offset to the share button, click once 

#         # 3) Share to Aruco Tag profile picture

#         # 4) Click again on Ipad to continue scrolling


#         # PART 3: Returning the Pencil to the Holder
#         # We should do this so we can continiously test without needing to reset after every every attempt.


#         # 1) Raise the Pencil

#         # 2) Move to Higher than Pre Grasp position(Same x,y different z)

#         # 3) Release the gripper
        
        
#         self.job_queue.append("toggle_grip")
#         """
        


#         ###OLD LAB CODE: NOt sure if I should delete
#         # joint_state = JointState()
#         # joint_state.name = self.joint_state.name[:]   
#         # joint_state.position = self.joint_state.position[:]   
#         # joint_state.position[2] = -3
#         # self.job_queue.append(joint_state)

#         # # 5) Move to release Position
#         # '''
#         # We want the release position to be 0.4m on the other side of the aruco tag relative to initial cube pose.
#         # Which offset will you change to achieve this and in what direction?
#         # '''
#         # joint_state = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + 0.4, self.cube_pose.point.y - 0.035, self.cube_pose.point.z + 0.185)
#         # self.job_queue.append(joint_state)


#         self.execute_jobs()


#     def execute_jobs(self):
#         if not self.job_queue:
#             self.get_logger().info("All jobs completed.")
#             rclpy.shutdown()
#             return

#         self.get_logger().info(f"Executing job queue, {len(self.job_queue)} jobs remaining.")
#         next_job = self.job_queue.pop(0)

#         if isinstance(next_job, JointState):
#             traj = self.ik_planner.plan_to_joints(next_job)
#             print(traj, next_job, JointState)
#             if traj is None:
#                 self.get_logger().error("Failed to plan to position")
#                 return

#             self.get_logger().info("Planned to position")
#             self._execute_joint_trajectory(traj.joint_trajectory)

#         elif next_job == 'toggle_grip':
#             self.get_logger().info("Toggling gripper")
#             self._toggle_gripper()
        
#         else:
#             self.get_logger().error("Unknown job type.")
#             self.execute_jobs()  # Proceed to next job

#     def _toggle_gripper(self):
#         if not self.gripper_cli.wait_for_service(timeout_sec=5.0):
#             self.get_logger().error('Gripper service not available')
#             rclpy.shutdown()
#             return

#         req = Trigger.Request()
#         future = self.gripper_cli.call_async(req)
#         # wait for 2 seconds
#         rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

#         self.get_logger().info('Gripper toggled.')
#         self.execute_jobs()  # Proceed to next job

            
#     def _execute_joint_trajectory(self, joint_traj):
#         self.get_logger().info('Waiting for controller action server...')
#         self.exec_ac.wait_for_server()

#         goal = FollowJointTrajectory.Goal()
#         goal.trajectory = joint_traj

#         self.get_logger().info('Sending trajectory to controller...')
#         send_future = self.exec_ac.send_goal_async(goal)
#         print(send_future)
#         send_future.add_done_callback(self._on_goal_sent)

#     def _on_goal_sent(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error('bonk')
#             rclpy.shutdown()
#             return

#         self.get_logger().info('Executing...')
#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self._on_exec_done)

#     def _on_exec_done(self, future):
#         try:
#             result = future.result().result
#             self.get_logger().info('Execution complete.')
#             self.execute_jobs()  # Proceed to next job
#         except Exception as e:
#             self.get_logger().error(f'Execution failed: {e}')


# def main(args=None):
#     rclpy.init(args=args)
#     node = UR7e_CubeGrasp()
#     rclpy.spin(node)
#     node.destroy_node()

# if __name__ == '__main__':
#     main()

# # ROS Libraries
# from std_srvs.srv import Trigger
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from control_msgs.action import FollowJointTrajectory
# from geometry_msgs.msg import PoseStamped # CHANGED: PointStamped -> PoseStamped
# from moveit_msgs.msg import RobotTrajectory
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from sensor_msgs.msg import JointState
# from tf2_ros import Buffer, TransformListener
# from scipy.spatial.transform import Rotation as R
# import numpy as np
# import time

# from planning.ik import IKPlanner

# class UR7e_CubeGrasp(Node):
#     def __init__(self):
#         super().__init__('cube_grasp')

#         # CHANGED: Subscribing to PoseStamped now
#         self.cube_sub = self.create_subscription(PoseStamped, '/cube_pose_in_base', self.cube_callback, 1) 
#         self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)

#         self.exec_ac = ActionClient(
#             self, FollowJointTrajectory,
#             '/scaled_joint_trajectory_controller/follow_joint_trajectory'
#         )

#         self.gripper_cli = self.create_client(Trigger, '/toggle_gripper')

#         self.cube_pose = None
#         self.current_plan = None
#         self.joint_state = None

#         self.ik_planner = IKPlanner()

#         self.job_queue = [] 

#     def joint_state_callback(self, msg: JointState):
#         self.joint_state = msg

#     def cube_callback(self, cube_pose_msg: PoseStamped):
#         if self.cube_pose is not None:
#             return

#         if self.joint_state is None:
#             self.get_logger().info("No joint state yet, cannot proceed")
#             return

#         print("Received cube pose.")
#         self.cube_pose = cube_pose_msg

#         # Extract position for cleaner code below
#         # CHANGED: Accessing .pose.position instead of .point
#         tx = self.cube_pose.pose.position.x
#         ty = self.cube_pose.pose.position.y
#         tz = self.cube_pose.pose.position.z

#         # -----------------------------------------------------------
#         # PART 1, PICK UP
#         # -----------------------------------------------------------

#         # 1) Move to Pre-Grasp Position (gripper above the pencil)
#         '''
#         Default values from lab 
#         x offset: 0.0
#         y offset: -0.035 
#         z offset: +0.185 (above cube)
#         '''
#         # Example logic using extracted variables
#         # Note: We are using the position from the tag, but usually keeping
#         # the gripper orientation fixed (downwards) via the IK planner defaults.
        
#         self.get_logger().info(f"Planning to: {tx}, {ty}, {tz}")

#         # Pre-grasp (High)
#         joint_state = self.ik_planner.compute_ik(self.joint_state, tx + 0.0 , ty - 0.172, tz + 0.25)
#         self.job_queue.append(joint_state)

#         # 2) Move to Grasp Position (Lower)
        
#         joint_state = self.ik_planner.compute_ik(self.joint_state, tx + 0.0, ty - 0.172, tz + 0.21)
#         self.job_queue.append(joint_state)

#         # 3) Close the gripper.
#         self.job_queue.append("toggle_grip")

#         # 4) Pick up pencil from the holder
#         joint_state = self.ik_planner.compute_ik(self.joint_state, tx + 0.0, ty - 0.172, tz + 0.4)
#         self.job_queue.append(joint_state)

#         # 5) Open Grip
#         self.job_queue.append("toggle_grip")
        
#         """
#         # 5) Navigate to the ipad
#         joint_state = self.ik_planner.compute_ik(self.joint_state, tx + 0.25, ty - 0.035, tz + 0.24)
#         self.job_queue.append(joint_state)
        
#         """

#         self.execute_jobs()

#     def execute_jobs(self):
#         if not self.job_queue:
#             self.get_logger().info("All jobs completed.")
#             # rclpy.shutdown() # Optional: keep node alive to receive new poses?
#             return

#         self.get_logger().info(f"Executing job queue, {len(self.job_queue)} jobs remaining.")
#         next_job = self.job_queue.pop(0)

#         if isinstance(next_job, JointState):
#             traj = self.ik_planner.plan_to_joints(next_job)
            
#             if traj is None:
#                 self.get_logger().error("Failed to plan to position")
#                 return

#             self.get_logger().info("Planned to position")
#             self._execute_joint_trajectory(traj.joint_trajectory)

#         elif next_job == 'toggle_grip':
#             self.get_logger().info("Toggling gripper")
#             self._toggle_gripper()
        
#         else:
#             self.get_logger().error("Unknown job type.")
#             self.execute_jobs()  # Proceed to next job

#     def _toggle_gripper(self):
#         if not self.gripper_cli.wait_for_service(timeout_sec=5.0):
#             self.get_logger().error('Gripper service not available')
#             rclpy.shutdown()
#             return

#         req = Trigger.Request()
#         future = self.gripper_cli.call_async(req)
#         rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

#         self.get_logger().info('Gripper toggled.')
#         self.execute_jobs()  # Proceed to next job

#     def _execute_joint_trajectory(self, joint_traj):
#         self.get_logger().info('Waiting for controller action server...')
#         self.exec_ac.wait_for_server()

#         goal = FollowJointTrajectory.Goal()
#         goal.trajectory = joint_traj

#         self.get_logger().info('Sending trajectory to controller...')
#         send_future = self.exec_ac.send_goal_async(goal)
#         send_future.add_done_callback(self._on_goal_sent)

#     def _on_goal_sent(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error('Goal rejected.')
#             rclpy.shutdown()
#             return

#         self.get_logger().info('Executing...')
#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self._on_exec_done)

#     def _on_exec_done(self, future):
#         try:
#             result = future.result().result
#             self.get_logger().info('Execution complete.')
#             self.execute_jobs()  # Proceed to next job
#         except Exception as e:
#             self.get_logger().error(f'Execution failed: {e}')


# def main(args=None):
#     rclpy.init(args=args)
#     node = UR7e_CubeGrasp()
#     rclpy.spin(node)
#     node.destroy_node()

# if __name__ == '__main__':
#     main()

# ROS Libraries
from std_srvs.srv import Trigger
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped 
from moveit_msgs.msg import RobotTrajectory # Unused import, but kept for completeness
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # Unused imports, but kept for completeness
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener # Unused imports, but kept for completeness
from scipy.spatial.transform import Rotation as R # Unused import, but kept for completeness
import numpy as np # Unused import, but kept for completeness
import time # Unused import, but kept for completeness


'''
ros2 launch planning lab7_bringup.launch.py plane_a:=0.0 plane_b:=0.0 plane_c:=0.0 plane_d:=0.0 ar_marker:=ar_marker_10
ros2 run planning main
'''

from planning.ik import IKPlanner

class UR7e_CubePickAndPlace(Node): # CHANGED: Class name
    def __init__(self):
        super().__init__('cube_pick_and_place') # CHANGED: Node name

        # Subscription for the CUBE's pickup pose
        self.cube_sub = self.create_subscription(PoseStamped, '/cube_pose_in_base', self.cube_pose_callback, 1) 
        
        # NEW Subscription for the DROP-OFF pose
        self.drop_off_sub = self.create_subscription(PoseStamped, '/drop_off_pose_in_base', self.drop_off_pose_callback, 1)

        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)

        self.exec_ac = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.gripper_cli = self.create_client(Trigger, '/toggle_gripper')

        self.cube_pose = None
        self.drop_off_pose = None # NEW: Store the drop-off pose
        self.joint_state = None
        
        # State flag to ensure the pick-and-place operation only runs once
        self.job_started = False 

        self.ik_planner = IKPlanner()
        self.job_queue = [] 

    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg

    def drop_off_pose_callback(self, drop_off_pose_msg: PoseStamped):
        """ NEW: Callback for the drop-off location pose. """
        if self.drop_off_pose is not None:
            return

        self.get_logger().info("Received drop-off pose.")
        self.drop_off_pose = drop_off_pose_msg
        
        # Check if both poses are ready to start the job
        self.check_and_start_job()


    def cube_pose_callback(self, cube_pose_msg: PoseStamped):
        """ Original Cube Pose callback, now updated to check for both poses. """
        if self.cube_pose is not None:
            return

        self.get_logger().info("Received cube pose.")
        self.cube_pose = cube_pose_msg
        
        # Check if both poses are ready to start the job
        self.check_and_start_job()

    def check_and_start_job(self):
        """ Triggers the job queue creation once all prerequisites are met. """
        if self.job_started:
            return

        if self.joint_state is None:
            self.get_logger().info("No joint state yet.")
            return

        if self.cube_pose is None or self.drop_off_pose is None:
            self.get_logger().info("Waiting for both cube and drop-off poses.")
            return

        self.job_started = True
        self.get_logger().info("All poses received. Starting Pick-and-Place sequence.")
        self.create_pick_and_place_jobs()
        self.execute_jobs()


    def create_pick_and_place_jobs(self):
        """ Defines the complete sequence of robot movements for pick and place. """
        
        # --- CUBE PICKUP SEQUENCE (from original code) ---
        
        # Cube Coordinates (Pick-up)
        # Note: Added offsets for gripper position relative to marker origin
        tx_cube = self.cube_pose.pose.position.x
        ty_cube = self.cube_pose.pose.position.y
        tz_cube = self.cube_pose.pose.position.z

        # Common offset for the UR7e gripper (adjusted from original for a cube):
        # The 0.172 offset on the Y-axis seems large for a cube, but I'll keep the
        # structure and tune the values slightly for a hypothetical grasping setup.

        self.get_logger().info(f"PICKUP | Cube: ({tx_cube:.3f}, {ty_cube:.3f}, {tz_cube:.3f})")

        # 1) Move to Pre-Grasp Position (High)
        joint_state = self.ik_planner.compute_ik(self.joint_state, tx_cube + 0.033, ty_cube + 0.01, tz_cube + 0.27)
        self.job_queue.append(joint_state)

        # 2) Move to Grasp Position (Lower)
        joint_state = self.ik_planner.compute_ik(self.joint_state, tx_cube + 0.033, ty_cube + 0.01, tz_cube + 0.23)
        self.job_queue.append(joint_state)

        # 3) Close the gripper (Grasp)
        self.job_queue.append("toggle_grip")

        # 4) Pick up cube (Lift to high position)
        joint_state = self.ik_planner.compute_ik(self.joint_state, tx_cube + 0.03, ty_cube + 0.01, tz_cube + 0.30)
        self.job_queue.append(joint_state)
        

        # Drop-off Coordinates
        tx_drop = self.drop_off_pose.pose.position.x
        ty_drop = self.drop_off_pose.pose.position.y
        tz_drop = self.drop_off_pose.pose.position.z

        self.get_logger().info(f"PLACE  | Drop: ({tx_drop:.3f}, {ty_drop:.3f}, {tz_drop:.3f})")
        
        # 5) Move to Drop-off Pre-Grasp Position (High above the target)
        # Uses the Drop-off X/Y and the LIFT_HEIGHT Z from pickup
        joint_state = self.ik_planner.compute_ik(self.joint_state, tx_drop, ty_drop, tz_drop + 0.35)
        self.job_queue.append(joint_state)
        
        # 6.1) FIRST SCROLL
        joint_state = self.ik_planner.compute_ik(self.joint_state, tx_drop, ty_drop, tz_drop + 0.23)
        self.job_queue.append(joint_state)
        joint_state = self.ik_planner.compute_ik(self.joint_state, tx_drop , ty_drop - 0.2, tz_drop + 0.23)
        self.job_queue.append(joint_state)

        # # 6.2) SECOND SCROLL
        # joint_state = self.ik_planner.compute_ik(self.joint_state, tx_drop, ty_drop, tz_drop + 0.35)
        # self.job_queue.append(joint_state)
        # joint_state = self.ik_planner.compute_ik(self.joint_state, tx_drop, ty_drop, tz_drop + 0.231)
        # self.job_queue.append(joint_state)
        # joint_state = self.ik_planner.compute_ik(self.joint_state, tx_drop , ty_drop - 0.2, tz_drop + 0.23)
        # self.job_queue.append(joint_state)

        # 7) Open the gripper (Release)
        self.job_queue.append("toggle_grip")

        # 8) Move Up from Drop-off Position (Retract)
        joint_state = self.ik_planner.compute_ik(self.joint_state, tx_drop, ty_drop , tz_drop + 0.35)
        self.job_queue.append(joint_state)
        
        # Note: The original step 5 "Open Grip" after pickup is removed, as it would drop the cube prematurely.
        # The commented-out 'Navigate to the ipad' step is replaced by the more structured drop-off sequence (steps 5-8).


    # --- Execution and Helper methods are the same ---
    
    def execute_jobs(self):
        if not self.job_queue:
            self.get_logger().info("All jobs completed.")
            # Optional: self.cube_pose = None, self.drop_off_pose = None, self.job_started = False 
            # to allow for a new pick-and-place cycle if new markers appear.
            return

        self.get_logger().info(f"Executing job queue, {len(self.job_queue)} jobs remaining.")
        next_job = self.job_queue.pop(0)

        if isinstance(next_job, JointState):
            traj = self.ik_planner.plan_to_joints(next_job)
            
            if traj is None:
                self.get_logger().error("Failed to plan to position")
                # Handle error: maybe attempt a different IK solution or abort
                return

            self.get_logger().info("Planned to position")
            self._execute_joint_trajectory(traj.joint_trajectory)

        elif next_job == 'toggle_grip':
            self.get_logger().info("Toggling gripper")
            self._toggle_gripper()
        
        else:
            self.get_logger().error("Unknown job type.")
            self.execute_jobs()  # Proceed to next job

    def _toggle_gripper(self):
        if not self.gripper_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Gripper service not available')
            # Changed: Do not shutdown on non-critical error, just log and proceed
            self.execute_jobs() 
            return

        req = Trigger.Request()
        future = self.gripper_cli.call_async(req)
        # Use a non-blocking check to avoid freezing the node if service is slow
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.done():
            self.get_logger().info('Gripper toggled.')
        else:
            self.get_logger().warn('Gripper toggle service timed out or failed.')

        self.execute_jobs()  # Proceed to next job

    def _execute_joint_trajectory(self, joint_traj):
        self.get_logger().info('Waiting for controller action server...')
        # Note: Waiting for server should typically be done once in init, but okay here for safety
        self.exec_ac.wait_for_server() 
        self.get_logger().info('debug')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj

        self.get_logger().info('Sending trajectory to controller...')
        send_future = self.exec_ac.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_sent)

    def _on_goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected. Aborting current plan.')
            # Abort the remaining jobs in the queue
            self.job_queue = [] 
            return

        self.get_logger().info('Executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_exec_done)

    def _on_exec_done(self, future):
        try:
            # Check result to see if execution was successful
            result = future.result().result
            self.get_logger().info('Execution complete.')
            self.execute_jobs()  # Proceed to next job
        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}. Aborting plan.')
            self.job_queue = [] # Abort the remaining jobs in the queue


def main(args=None):
    rclpy.init(args=args)
    node = UR7e_CubePickAndPlace()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()