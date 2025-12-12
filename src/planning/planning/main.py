# ROS Libraries
from std_srvs.srv import Trigger
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped 
from moveit_msgs.msg import RobotTrajectory 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint 
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener 
from scipy.spatial.transform import Rotation as R 
import numpy as np 
import time 

'''
ros2 launch planning lab7_bringup.launch.py plane_a:=0.0 plane_b:=0.0 plane_c:=0.0 plane_d:=0.0 ar_marker:=ar_marker_10
ros2 run planning main
'''

from planning.ik import IKPlanner

class UR7e_CubePickAndPlace(Node):
    def __init__(self):
        super().__init__('cube_pick_and_place')

        # Subscription for the CUBE's pickup pose (Tag 1)
        self.cube_sub = self.create_subscription(PoseStamped, '/cube_pose_in_base', self.cube_pose_callback, 1) 
        
        # Subscription for the DROP-OFF/SCROLL pose (Tag 2)
        self.drop_off_sub = self.create_subscription(PoseStamped, '/drop_off_pose_in_base', self.drop_off_pose_callback, 1)

        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)

        # REMOVED: Calibration Tag 3 subscription

        self.exec_ac = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.gripper_cli = self.create_client(Trigger, '/toggle_gripper')

        self.cube_pose = None
        self.drop_off_pose = None
        # REMOVED: self.calib_pose
        self.joint_state = None
        
        # State flag to ensure the pick-and-place operation only runs once
        self.job_started = False 

        self.ik_planner = IKPlanner()
        self.job_queue = [] 

    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg

    def drop_off_pose_callback(self, drop_off_pose_msg: PoseStamped):
        """ Callback for the drop-off location pose (Tag 2). """
        if self.drop_off_pose is not None:
            return

        self.get_logger().info("Received drop-off pose (Tag 2).")
        self.drop_off_pose = drop_off_pose_msg
        
        self.check_and_start_job()

    def cube_pose_callback(self, cube_pose_msg: PoseStamped):
        """ Cube Pose callback (Tag 1). """
        if self.cube_pose is not None:
            return

        self.get_logger().info("Received cube pose (Tag 1).")
        self.cube_pose = cube_pose_msg
        
        self.check_and_start_job()

    def check_and_start_job(self):
        if self.job_started: 
            return
        if self.joint_state is None: 
            return

        # UPDATED: Only check for cube_pose (1) and drop_off_pose (2)
        if self.cube_pose is None or self.drop_off_pose is None:
            return

        self.job_started = True
        self.get_logger().info("Both Poses (Tag 1 & 2) Received. Starting Sequence.")
        self.create_pick_and_place_jobs()
        self.execute_jobs()


    def create_pick_and_place_jobs(self):
        """ Phase A: Pickup (Tag 1) then Move to Calibration Point (Tag 2). """
        
        # ---------------------------------------------------------
        # PART 1: CUBE PICKUP (Tag 1)
        # ---------------------------------------------------------
        tx_cube = self.cube_pose.pose.position.x - 0.01
        ty_cube = self.cube_pose.pose.position.y
        tz_cube = self.cube_pose.pose.position.z

        print(f"Pick up Tag 1 at: x={tx_cube:.3f}, y={ty_cube:.3f}, z={tz_cube:.3f}")
        self.get_logger().info("Planning Pickup (Tag 1)...")

        # 1) Pre-Grasp
        self.job_queue.append(self.ik_planner.compute_ik(
            self.joint_state, tx_cube, ty_cube - 0.03, tz_cube + 0.25))
        
        # 2) Grasp
        self.job_queue.append(self.ik_planner.compute_ik(
            self.joint_state, tx_cube, ty_cube - 0.03, tz_cube + 0.185))
        
        # 3) Close
        self.job_queue.append("toggle_grip")

        # 4) Retract
        self.job_queue.append(self.ik_planner.compute_ik(
            self.joint_state, tx_cube, ty_cube - 0.03, tz_cube + 0.35))


        # ---------------------------------------------------------
        # PART 2: MOVE TO CALIBRATION ZONE (Above Tag 2)
        # ---------------------------------------------------------
        # Now we move directly to Tag 2 to do the calibration
        tx_drop = self.drop_off_pose.pose.position.x
        ty_drop = self.drop_off_pose.pose.position.y
        tz_drop = self.drop_off_pose.pose.position.z

        print(f"Moving to Drop-off Tag 2 at: x={tx_drop:.3f}, y={ty_drop:.3f}, z={tz_drop:.3f}")
        self.get_logger().info("Positioning for Calibration above Tag 2...")

        # 5) Hover High over Tag 2 (Safety)
        hover_z = tz_drop + 0.35
        self.job_queue.append(self.ik_planner.compute_ik(
            self.joint_state, tx_drop, ty_drop, hover_z))

        # 6) Move to "Starting Height" for the interactive test
        # We start at +0.34 relative to Tag 2
        start_height = 0.34
        self.job_queue.append(self.ik_planner.compute_ik(
            self.joint_state, tx_drop, ty_drop, tz_drop + start_height))
        
        # ---------------------------------------------------------
        # TRIGGER INTERACTIVE MODE
        # ---------------------------------------------------------
        self.job_queue.append("GET_USER_INPUT")

        # 6) SCROLL SEQUENCE (Loop 5 times)
    def plan_scrolling_sequence(self, z_offset):
        """ 
        Phase B: Plans the Scrolling (Tag 2).
        Uses the z_offset confirmed by the user in the previous step.
        """
        
        # ---------------------------------------------------------
        # PART 3: SCROLL SEQUENCE (Tag 2)
        # ---------------------------------------------------------
        tx_drop = self.drop_off_pose.pose.position.x - 0.035
        ty_drop = self.drop_off_pose.pose.position.y
        tz_drop = self.drop_off_pose.pose.position.z

        self.get_logger().info(f"Planning Scroll with calibrated offset: {z_offset}")

        touch_z = tz_drop + z_offset 
        hover_z = touch_z + 0.05 

        scroll_start_y = ty_drop + 0.1
        scroll_end_y = ty_drop - 0.1

        joint_state = self.ik_planner.compute_ik(self.joint_state, tx_drop, ty_drop, hover_z)

    

        print(f"  Scroll from y={scroll_start_y:.3f} to y={scroll_end_y:.3f}")
        for i in range(5):
            self.get_logger().info(f"Programming Scroll {i+1}/5")

            joint_state = self.ik_planner.compute_ik(self.joint_state, tx_drop, scroll_start_y, hover_z)
            self.job_queue.append(joint_state)


            # B. TOUCH (Push in Z)
            joint_state = self.ik_planner.compute_ik(self.joint_state, tx_drop, scroll_start_y, touch_z)
            self.job_queue.append(joint_state)

            # C. DRAG (Move Y down)
            joint_state = self.ik_planner.compute_ik(self.joint_state,  tx_drop, scroll_end_y, touch_z)
            self.job_queue.append(joint_state)


            joint_state = self.ik_planner.compute_ik(self.joint_state, tx_drop, scroll_end_y, hover_z)
            self.job_queue.append(joint_state)



        joint_state = self.ik_planner.compute_ik(self.joint_state, tx_drop, scroll_end_y, tz_drop + 0.40)
        self.job_queue.append(joint_state)

        self.job_queue.append("toggle_grip") 
        self.execute_jobs()

    # --- Execution and Helper methods ---
    
    def execute_jobs(self):
        if not self.job_queue:
            self.get_logger().info("All jobs completed.")
            return

        self.get_logger().info(f"Executing job queue, {len(self.job_queue)} jobs remaining.")
        next_job = self.job_queue.pop(0)

        if isinstance(next_job, JointState):
            traj = self.ik_planner.plan_to_joints(next_job)
            
            if traj is None:
                self.get_logger().error("Failed to plan to position")
                return

            self.get_logger().info("Planned to position")
            self._execute_joint_trajectory(traj.joint_trajectory)

        elif next_job == 'toggle_grip':
            self.get_logger().info("Toggling gripper")
            self._toggle_gripper()
        

        elif next_job == "GET_USER_INPUT":
            # Start at the safe guess we moved to earlier
            current_offset = 0.34
            step_size = 0.0025  

            print("\n" + "="*60)
            print("  INTERACTIVE CALIBRATION MODE (ABOVE TAG 2)")
            print(f"  Current Offset: {current_offset:.4f}")
            print("  [w] = UP (+2.5mm)")
            print("  [s] = DOWN (-2.5mm)")
            print("  [true] = CONFIRM and START SCROLL")
            print("="*60 + "\n")

            last_stable_config = self.joint_state

            # Helper to move without drift
            def move_stabilized(target_offset):
                nonlocal last_stable_config 
                
                # UPDATED: Use self.drop_off_pose (Tag 2) as reference
                tx = self.drop_off_pose.pose.position.x
                ty = self.drop_off_pose.pose.position.y
                tz = self.drop_off_pose.pose.position.z

                print(f"  Moving to Z={tz + target_offset:.4f} (Offset: {target_offset:.4f})")

                # 1. Use the LAST STABLE CONFIG as the seed
                ik_sol = self.ik_planner.compute_ik(last_stable_config, tx, ty, tz + target_offset)
                
                # 2. Update our "Stable Config"
                if ik_sol:
                    last_stable_config = ik_sol
                    
                    # 3. Move the robot physically
                    traj = self.ik_planner.plan_to_joints(ik_sol)
                    if traj:
                        self._execute_joint_trajectory(traj.joint_trajectory)
                        time.sleep(2.0) # Wait for physical move
                    else:
                        print("  ! TRAJ PLAN FAILED !")
                else:
                    print("  ! IK SOLUTION FAILED !")

            while True:
                user_cmd = input(f"Offset {current_offset:.4f} | Cmd: ").strip().lower()

                if user_cmd in ['true', 'y', 'ok']:
                    print(f"CONFIRMED: {current_offset:.4f}. Starting Scroll...")
                    # Pass the calibrated offset to the scrolling planner
                    self.plan_scrolling_sequence(current_offset)
                    return 

                elif user_cmd in ['w', 'up']:
                    current_offset += step_size
                    print(f"  -> UP to {current_offset:.4f}")
                    move_stabilized(current_offset)

                elif user_cmd in ['s', 'down']:
                    current_offset -= step_size
                    print(f"  -> DOWN to {current_offset:.4f}")
                    move_stabilized(current_offset)
        
        else:
            self.get_logger().error("Unknown job type.")
            self.execute_jobs()  # Proceed to next job

    def _toggle_gripper(self):
        if not self.gripper_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Gripper service not available')
            self.execute_jobs() 
            return

        req = Trigger.Request()
        future = self.gripper_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.done():
            self.get_logger().info('Gripper toggled.')
        else:
            self.get_logger().warn('Gripper toggle service timed out or failed.')

        self.execute_jobs()  

    def _execute_joint_trajectory(self, joint_traj):
        self.get_logger().info('Waiting for controller action server...')
        self.exec_ac.wait_for_server() 
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj

        self.get_logger().info('Sending trajectory to controller...')
        send_future = self.exec_ac.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_sent)

    def _on_goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected. Aborting current plan.')
            self.job_queue = [] 
            return

        self.get_logger().info('Executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_exec_done)

    def _on_exec_done(self, future):
        try:
            result = future.result().result
            self.get_logger().info('Execution complete.')
            self.execute_jobs()  # Proceed to next job
        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}. Aborting plan.')
            self.job_queue = [] 


def main(args=None):
    rclpy.init(args=args)
    node = UR7e_CubePickAndPlace()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()