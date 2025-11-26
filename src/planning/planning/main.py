# ROS Libraries
from std_srvs.srv import Trigger
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped 
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
import numpy as np
import time

from planning.ik import IKPlanner

class UR7e_CubeGrasp(Node):
    def __init__(self):
        super().__init__('cube_grasp')

        self.cube_pub = self.create_subscription(PointStamped, '/cube_pose_in_base', self.cube_callback, 1) # TODO: CHECK IF TOPIC ALIGNS WITH YOURS
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)

        self.exec_ac = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.gripper_cli = self.create_client(Trigger, '/toggle_gripper')

        self.cube_pose = None
        self.current_plan = None
        self.joint_state = None

        self.ik_planner = IKPlanner()

        self.job_queue = [] # Entries should be of type either JointState or String('toggle_grip')

    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg

    def cube_callback(self, cube_pose):
        print("cube callback")
        if self.cube_pose is not None:
            return

        if self.joint_state is None:
            self.get_logger().info("No joint state yet, cannot proceed")
            return

        self.cube_pose = cube_pose


        # -----------------------------------------------------------
        # Code called to Pick up pencil from holder and navigate to the IPad and begin scrolling motion
        # Should work with Aruco Tags instead of the current vision cloud.
        # -----------------------------------------------------------

        # PART 1, PICK UP

        # 1) Move to Pre-Grasp Position (gripper above the pencil)
        '''
        Default values from lab 
        Use the following offsets for pre-grasp position:
        x offset: 0.0
        y offset: -0.035 
        z offset: +0.185 (to be above the cube by accounting for gripper length)
        '''
        joint_state = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + 0.0, self.cube_pose.point.y - 0.035, self.cube_pose.point.z + 0.35)
        self.job_queue.append(joint_state)

        # 2) Move to Grasp Position (lower the gripper to the pencil)
        '''
        Note that this will again be defined relative to the cube pose. 
        DO NOT CHANGE z offset lower than +0.16. 
        '''
        joint_state = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + 0.0, self.cube_pose.point.y - 0.035, self.cube_pose.point.z + 0.21)

        self.job_queue.append(joint_state)


        # 3) Close the gripper.
        self.job_queue.append("toggle_grip")

        # 4) Pick up pencil from the holder
        joint_state = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + 0.0, self.cube_pose.point.y - 0.035, self.cube_pose.point.z + 0.4)

        self.job_queue.append(joint_state)
        
        # 5) Navigate to the ipad
        #TODO: UTilize Ipad Aurco Tags
        joint_state = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + 0.25, self.cube_pose.point.y - 0.035, self.cube_pose.point.z + 0.24)
        self.job_queue.append(joint_state)

        # PART 2, IPAD


        # 1) Scroling Motion
        #TODO: Make a function to scroll continiously or for x iterations
        joint_state = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + 0.25, self.cube_pose.point.y - 0.035, self.cube_pose.point.z + 0.239)
        self.job_queue.append(joint_state)

        joint_state = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + 0.25, self.cube_pose.point.y, self.cube_pose.point.z + 0.239)
        self.job_queue.append(joint_state)

        # 2) Move to Share button
        #Probbaly code a prexisting offset to the share button, click once 

        # 3) Share to Aruco Tag profile picture

        # 4) Click again on Ipad to continue scrolling


        # PART 3: Returning the Pencil to the Holder
        # We should do this so we can continiously test without needing to reset after every every attempt.


        # 1) Raise the Pencil

        # 2) Move to Higher than Pre Grasp position(Same x,y different z)

        # 3) Release the gripper
        self.job_queue.append("toggle_grip")

        


        ###OLD LAB CODE: NOt sure if I should delete
        # joint_state = JointState()
        # joint_state.name = self.joint_state.name[:]   
        # joint_state.position = self.joint_state.position[:]   
        # joint_state.position[2] = -3
        # self.job_queue.append(joint_state)

        # # 5) Move to release Position
        # '''
        # We want the release position to be 0.4m on the other side of the aruco tag relative to initial cube pose.
        # Which offset will you change to achieve this and in what direction?
        # '''
        # joint_state = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + 0.4, self.cube_pose.point.y - 0.035, self.cube_pose.point.z + 0.185)
        # self.job_queue.append(joint_state)


        self.execute_jobs()


    def execute_jobs(self):
        if not self.job_queue:
            self.get_logger().info("All jobs completed.")
            rclpy.shutdown()
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
        
        else:
            self.get_logger().error("Unknown job type.")
            self.execute_jobs()  # Proceed to next job

    def _toggle_gripper(self):
        if not self.gripper_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Gripper service not available')
            rclpy.shutdown()
            return

        req = Trigger.Request()
        future = self.gripper_cli.call_async(req)
        # wait for 2 seconds
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        self.get_logger().info('Gripper toggled.')
        self.execute_jobs()  # Proceed to next job

            
    def _execute_joint_trajectory(self, joint_traj):
        self.get_logger().info('Waiting for controller action server...')
        self.exec_ac.wait_for_server()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj

        self.get_logger().info('Sending trajectory to controller...')
        send_future = self.exec_ac.send_goal_async(goal)
        print(send_future)
        send_future.add_done_callback(self._on_goal_sent)

    def _on_goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('bonk')
            rclpy.shutdown()
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
            self.get_logger().error(f'Execution failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UR7e_CubeGrasp()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
