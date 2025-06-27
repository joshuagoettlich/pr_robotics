#!/usr/bin/env python3

import rospy
import math
import numpy as np
import sys
import os
import time
from typing import Tuple, Optional

# --- Import for ROS Controller Messages ---
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

# --- Imports from Custom Local Modules ---
# These modules are expected to be in the same directory or in the Python path.
try:
    from simple_ik import solve_ik_with_closest_solution, RobotKinematics
    from extract_traj import extract_trajectories_from_dxf_in_meters, plot_trajectories
except ImportError as e:
    print(f"CRITICAL: Failed to import local modules (simple_ik, extract_traj).")
    print(f"Error: {e}")
    print("Please ensure 'simple_ik.py' and 'extract_traj.py' are in the same directory.")
    sys.exit(1)


# ==============================================================================
# SECTION 1: ROBOT CONTROLLER
# This class now uses the imported functions for its logic.
# ==============================================================================

class IKRobotController:
    def __init__(self):
        """Initializes the robot controller node."""
        rospy.init_node('ik_robot_controller', anonymous=True)
        
        # --- Use the controller command topic and message type ---
        self.controller_command_topic = '/three_link_arm_controller/command'
        self.joint_pub = rospy.Publisher(self.controller_command_topic, JointTrajectory, queue_size=10)
        
        self.rate = rospy.Rate(50) # Loop rate for trajectory execution

        # Instantiate the kinematics model from the imported module
        self.kinematics = RobotKinematics()

        # Define a safe starting/home configuration in joint space (q1, q2)
        self.home_q_rad = (0.0, math.radians(45))
        self.current_q_rad = self.home_q_rad # Assume starting at home
        self.prismatic_joint_pos = 0.0  # Keep prismatic joint fixed

        rospy.loginfo(f"Robot IK Controller initialized.")
        rospy.loginfo(f"Publishing commands to: {self.controller_command_topic}")

    def wait_for_controller(self):
        """Waits for the joint trajectory controller to be ready."""
        rospy.loginfo("Waiting for the controller to be ready...")
        while self.joint_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)
        if rospy.is_shutdown():
            rospy.logerr("ROS shutdown while waiting for controller.")
            return False
        rospy.loginfo("Controller is ready.")
        return True

    def send_joint_goal(self, q1_rad, q2_rad, duration):
        """
        Creates and sends a JointTrajectory message to command the robot.
        """
        traj_msg = JointTrajectory()
        traj_msg.header = Header(stamp=rospy.Time.now())
        traj_msg.joint_names = ['joint1', 'joint2', 'joint3'] # Must match controller config

        point = JointTrajectoryPoint()
        point.positions = [q1_rad, q2_rad, self.prismatic_joint_pos]
        point.time_from_start = rospy.Duration(duration)
        
        traj_msg.points.append(point)
        
        self.joint_pub.publish(traj_msg)
        self.current_q_rad = (q1_rad, q2_rad) # Update internal state

    def move_to_home(self, duration=3.0):
        """Commands the robot to its predefined home position."""
        rospy.loginfo(f"Moving to HOME position ({math.degrees(self.home_q_rad[0]):.1f}°, {math.degrees(self.home_q_rad[1]):.1f}°)...")
        self.send_joint_goal(self.home_q_rad[0], self.home_q_rad[1], duration)
        rospy.sleep(duration) # Wait for the move to complete
        rospy.loginfo("Homing complete.")

    def execute_trajectory(self, trajectory_points_local):
        """
        Calculates IK for and executes a given trajectory point by point.
        """
        if not trajectory_points_local.any():
            rospy.logwarn("Cannot execute an empty trajectory.")
            return

        # Start the trajectory from the robot's current position
        start_xy = self.kinematics.forward_kinematics_2d(*self.current_q_rad)
        rospy.loginfo(f"Current end-effector position (x,y): {np.round(start_xy, 3)}. Offsetting trajectory to start here.")

        global_trajectory = trajectory_points_local + start_xy
        
        # Optional plot using the imported function
        plot_trajectories([global_trajectory])
        
        # Duration for each small segment of the trajectory
        segment_duration = 0.1 # seconds

        rospy.loginfo(f"Executing trajectory with {len(global_trajectory)} points...")
        for point in global_trajectory:
            if rospy.is_shutdown():
                break

            target_xy_rel_to_joint1 = self.kinematics.get_target_relative_to_joint1(point)

            # Use the imported IK solver
            q_solution = solve_ik_with_closest_solution(
                L1=self.kinematics.L1, L1_offset=self.kinematics.L1_offset,
                L2=self.kinematics.L2, L2_offset=self.kinematics.L2_offset,
                x=target_xy_rel_to_joint1[0], y=target_xy_rel_to_joint1[1],
                current_q_rad=self.current_q_rad
            )

            if q_solution:
                self.send_joint_goal(q_solution[0], q_solution[1], segment_duration)
                rospy.sleep(segment_duration) # Wait for the segment to finish
            else:
                rospy.logwarn_throttle(2, f"IK solution not found for point {np.round(point, 3)}. Skipping.")
            
        rospy.loginfo("Trajectory execution finished.")

def wait_for_enter(prompt_message):
    """Pauses execution and waits for the user to press Enter."""
    rospy.loginfo(prompt_message)
    try:
        input() # Python 3
    except NameError:
        raw_input() # Python 2


# ==============================================================================
# SECTION 2: MAIN EXECUTION BLOCK
# ==============================================================================

if __name__ == '__main__':
    try:
        controller = IKRobotController()
        
        # Wait for Gazebo/controller to be ready before proceeding
        if not controller.wait_for_controller():
             sys.exit(1)
        
        # Get DXF file path from command line argument or user input
        if len(sys.argv) > 1:
            dxf_filepath = sys.argv[1]
        else:
            dxf_filepath = input("Please enter the full path to your DXF file: ")

        if not os.path.isfile(dxf_filepath):
            rospy.logerr(f"File not found: '{dxf_filepath}'")
            sys.exit(1)

        # Extract Trajectories using the imported function
        trajectories = extract_trajectories_from_dxf_in_meters(dxf_filepath)
        if not trajectories:
            rospy.logerr("No trajectories were extracted. Exiting.")
            sys.exit(1)

        # Robot Execution Flow
        wait_for_enter(f"\nPress Enter to move the robot to its HOME position.")
        controller.move_to_home()

        wait_for_enter("Press Enter to execute the first trajectory from the DXF file.")
        controller.execute_trajectory(trajectories[0])
        controller.execute_trajectory(trajectories[1])
        
        rospy.loginfo("Script finished successfully.")

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")
    except KeyboardInterrupt:
        rospy.loginfo("Script terminated by user (Ctrl+C).")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {e}", exc_info=True)
