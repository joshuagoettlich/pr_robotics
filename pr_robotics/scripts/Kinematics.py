#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ==============================================================================
# SECTION 1: IMPORTS
# ==============================================================================
import rospy
import math
import numpy as np
import sys
import os
import tty
import termios
import time
import threading
from typing import Tuple, Optional
from simple_ik import *
# --- ROS-related imports ---
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# --- Dynamixel SDK import ---
from dynamixel_sdk import *
from controll_class import *
# --- Dependency Checks for Optional Libraries ---
EZDXF_AVAILABLE = False
try:
    import ezdxf
    EZDXF_AVAILABLE = True
except ImportError:
    print("The 'ezdxf' library is required for DXF parsing. Please run: pip install ezdxf")

MATPLOTLIB_AVAILABLE = False
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    print("The 'matplotlib' library is recommended for trajectory visualization. Please run: pip install matplotlib")
from extract_traj import extract_trajectories_from_dxf_in_meters, plot_trajectories

from simpl
# ==============================================================================
# SECTION 2: THE DYNAMIXEL CONTROLLER CLASS
# (This class is now thread-safe and stable)
# ==============================================================================


# ==============================================================================
# SECTION 3: UTILITY AND KINEMATICS
# (DXF parsing function is now fixed)
# ==============================================================================


# --- Robot Kinematics and Joint Conversions (Unchanged) ---
DXL_IDS = [1, 2, 3]
POSITION_LIMITS = [(1100, 2900), (1100, 3000), (2000, 3270)]
PRISMATIC_MIN_M, PRISMATIC_MAX_M = 0.0, 0.05

# ==============================================================================
# SECTION 4: ROS INTERFACE CLASS
# (This class is stable)
# ==============================================================================

class DynamixelRosInterface:
    def __init__(self, device_name, baudrate, z_axis_id, z_up_pos, z_down_pwm):
        self.dxl_controller = DynamixelController(
            ids=DXL_IDS, z_axis_id=z_axis_id, z_up_position=z_up_pos,
            z_down_pwm=z_down_pwm, position_limits=POSITION_LIMITS,
            device_name=device_name, baudrate=baudrate
        )
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.joint_state_msg = JointState(name=[f"joint{i}" for i in DXL_IDS])
        self.state_publisher_timer = rospy.Timer(rospy.Duration(0.1), self.publish_current_state)

    def publish_current_state(self, event=None):
        if rospy.is_shutdown(): return
        positions_dxl = self.dxl_controller.read_positions()
        positions_rad_m = [dxl_units_to_rad(p, i) for i, p in enumerate(positions_dxl)]
        self.joint_state_msg.position = positions_rad_m
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.joint_state_msg)

    def get_current_joint_positions_rad(self) -> Optional[np.ndarray]:
        positions_dxl = self.dxl_controller.read_positions()
        if not positions_dxl or any(p is None for p in positions_dxl):
            rospy.logwarn("Failed to read initial joint positions.")
            return None
        return np.array([dxl_units_to_rad(p, i) for i, p in enumerate(positions_dxl)])

    def publish_xy_positions_dxl(self, positions_dxl_units: list):
        self.dxl_controller.write_positions([DXL_IDS[0], DXL_IDS[1]], positions_dxl_units)

    def wait_until_xy_reached(self, target_positions_dxl: list):
        self.dxl_controller.wait_until_reached([DXL_IDS[0], DXL_IDS[1]], target_positions_dxl)

    def lift_pen(self): self.dxl_controller.move_z_up()
    def lower_pen(self): self.dxl_controller.move_z_down()

    def shutdown(self):
        rospy.loginfo("Shutting down ROS interface...")
        if self.state_publisher_timer:
            self.state_publisher_timer.shutdown()
            rospy.loginfo("State publisher timer stopped.")
        if self.dxl_controller:
            self.dxl_controller.disable_torque_and_close()

# ==============================================================================
# SECTION 5: MAIN EXECUTION BLOCK
# ==============================================================================

if __name__ == "__main__":
    rospy.init_node('trajectory_follower_node', anonymous=True)
    ros_interface = None
    try:
        # --- Configuration ---
        DEVICE_NAME = "/dev/ttyUSB0"
        BAUDRATE = 1000000
        Z_AXIS_ID = 3
        Z_UP_POS_DXL = 3000
        Z_DOWN_PWM = 40
        DXF_PATH = "./Linien.dxf"

        robot_kinematics = RobotKinematics()
        ros_interface = DynamixelRosInterface(DEVICE_NAME, BAUDRATE, Z_AXIS_ID, Z_UP_POS_DXL, Z_DOWN_PWM)

        rospy.sleep(1.0)
        
        q_current = ros_interface.get_current_joint_positions_rad()
        if q_current is None: raise rospy.exceptions.ROSInterruptException("Could not read motor state.")
        
        robot_kinematics.set_initial_end_effector_pos(q_current[0], q_current[1])

        aligned_trajectories = extract_trajectories_from_dxf_in_meters(DXF_PATH)
        if not aligned_trajectories:
            rospy.logwarn("No trajectories extracted. Exiting node.")
            sys.exit(0)

        plot_trajectories(aligned_trajectories, offset=robot_kinematics.current_end_effector_xy_base_frame)
        rospy.loginfo("Starting trajectory following in 3 seconds. Press Ctrl+C to stop.")
        rospy.sleep(3.0)

        # --- Main Loop ---

        for traj_idx, traj in enumerate(aligned_trajectories):
            if rospy.is_shutdown(): break
            rospy.loginfo(f"--- Starting Trajectory {traj_idx + 1}/{len(aligned_trajectories)} ---")

            ros_interface.lift_pen(); rospy.sleep(0.5)

            start_point_world = traj[0] + robot_kinematics.current_end_effector_xy_base_frame
            start_point_rel_j1 = robot_kinematics.get_target_relative_to_joint1(start_point_world)
            ik_sol = solve_ik_with_closest_solution(robot_kinematics.L1,-0.031, robot_kinematics.L2,0.038, start_point_rel_j1[0], start_point_rel_j1[1],q_current)
            print(robot_kinematics.forward_kinematics_2d(ik_sol[0], ik_sol[1]))
            if not ik_sol:
                rospy.logwarn("IK solution failed for start point. Skipping trajectory."); continue

            q_dxl = [rad_to_dxl_units(q, i) for i, q in enumerate(ik_sol)]
            ros_interface.publish_xy_positions_dxl(q_dxl)
            ros_interface.wait_until_xy_reached(q_dxl); rospy.sleep(0.5)

            ros_interface.lower_pen(); rospy.sleep(0.5)

            last_valid_q_dxl = q_dxl
            for point in traj[1:]:
                if rospy.is_shutdown(): break
                point_world = point + robot_kinematics.current_end_effector_xy_base_frame
                print(point_world)
                point_rel_j1 = point_world

                ik_sol = solve_ik_with_closest_solution(robot_kinematics.L1, -0.031, robot_kinematics.L2, 0.038, point_rel_j1[0], point_rel_j1[1], current_q_rad=ik_sol)
                if ik_sol:
                    q_dxl = [rad_to_dxl_units(q, i) for i, q in enumerate(ik_sol)]
                    ros_interface.publish_xy_positions_dxl(q_dxl)
                    last_valid_q_dxl = q_dxl
                    rospy.sleep(0.05)

            ros_interface.wait_until_xy_reached(last_valid_q_dxl)
            rospy.loginfo(f"--- Finished Trajectory {traj_idx + 1} ---")

        ros_interface.lift_pen()
        rospy.loginfo("All trajectories finished.")

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted by user or error.")
    except Exception as e:
        rospy.logfatal(f"An unhandled exception occurred in main: {e}")
    finally:
        if ros_interface:
            rospy.loginfo("Initiating final shutdown sequence...")
            try:
                ros_interface.lift_pen()
                rospy.sleep(1.0)
                ros_interface.shutdown()
            except Exception as shutdown_e:
                rospy.logerr(f"Error during controlled shutdown: {shutdown_e}")
        rospy.loginfo("Node has shut down.")