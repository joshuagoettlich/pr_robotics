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

# --- ROS-related imports ---
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# --- Dynamixel SDK import ---
from dynamixel_sdk import *

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

# ==============================================================================
# SECTION 2: THE DYNAMIXEL CONTROLLER CLASS
# (This class is now thread-safe and stable)
# ==============================================================================

class DynamixelController:
    """
    A thread-safe controller class for Dynamixel motors, supporting both Position and PWM Control modes.
    """
    def __init__(self, ids, z_axis_id, z_up_position, z_down_pwm, position_limits=None, device_name='/dev/ttyUSB0', baudrate=1000000, protocol_version=2.0):
        assert isinstance(ids, list), "Motor IDs must be a list."
        assert z_axis_id in ids, "Z-axis ID must be one of the provided motor IDs."
        self.dxl_ids = ids
        self.z_axis_id = z_axis_id
        self.z_up_position = z_up_position
        self.z_down_pwm = z_down_pwm
        self.device_name = device_name
        self.baudrate = baudrate
        self.protocol_version = protocol_version

        # Control Table Addresses
        self.ADDR_OPERATING_MODE = 11
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_PRESENT_LOAD = 126
        self.ADDR_GOAL_PWM = 100
        self.ADDR_PROFILE_ACCELERATION = 108
        self.ADDR_PROFILE_VELOCITY = 112

        self.ADDR_POSITION_D_GAIN = 80
        self.ADDR_POSITION_I_GAIN = 82
        self.ADDR_POSITION_P_GAIN = 84

        # Operational Values
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        self.MOVING_THRESHOLD_POSITION = 1
        self.STOPPED_LOAD_THRESHOLD = 20

        # Operating Modes
        self.OPERATING_MODE_POSITION = 3
        self.OPERATING_MODE_PWM = 16

        if position_limits is None: self.position_limits = [(1100, 2900)] * len(ids)
        else: self.position_limits = position_limits

        self.current_modes = {dxl_id: None for dxl_id in self.dxl_ids}
        self.port_is_open = False
        self.comm_lock = threading.Lock()

        self.portHandler = PortHandler(self.device_name)
        self.packetHandler = PacketHandler(self.protocol_version)

        self._open_port()
        self._initial_setup()

    def _open_port(self):
        with self.comm_lock:
            if self.portHandler.openPort():
                self.port_is_open = True
            else:
                rospy.logfatal(f"Failed to open port {self.device_name}")
                sys.exit(1)
            if not self.portHandler.setBaudRate(self.baudrate):
                rospy.logfatal(f"Failed to set baudrate to {self.baudrate}")
                sys.exit(1)
            rospy.loginfo(f"Dynamixel port {self.device_name} opened at {self.baudrate} bps.")

    def _initial_setup(self):
        with self.comm_lock:
            rospy.loginfo("--- Performing initial setup for all motors ---")
            for dxl_id in self.dxl_ids:
                self.set_torque_enable(dxl_id, self.TORQUE_DISABLE, locked=True)
                self.set_operating_mode(dxl_id, self.OPERATING_MODE_POSITION, locked=True)
                self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PROFILE_ACCELERATION, 10)
                self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PROFILE_VELOCITY, 200)
                self.set_torque_enable(dxl_id, self.TORQUE_ENABLE, locked=True)
                self.set_pid_gains(dxl_id, 1000, 100, 100, locked=True)  # Example PID gains
            rospy.loginfo("--- Initial setup complete ---")

    def _check_comm(self, result, error, context=""):
        if result != COMM_SUCCESS:
            rospy.logwarn(f"COMM_ERROR: {context}: {self.packetHandler.getTxRxResult(result)}")
        elif error != 0:
            rospy.logwarn(f"PACKET_ERROR: {context}: {self.packetHandler.getRxPacketError(error)}")

    def set_torque_enable(self, dxl_id, enable, locked=False):
        if not locked: self.comm_lock.acquire()
        try:
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, enable)
            self._check_comm(result, error, f"Setting torque for ID {dxl_id}")
        finally:
            if not locked: self.comm_lock.release()

    def set_pid_gains(self, dxl_id, p_gain, i_gain, d_gain, locked=False):
        """Sets the Position P, I, and D gains for a specific motor."""
        if not locked: self.comm_lock.acquire()
        try:
            # It's good practice to disable torque before changing critical parameters
            self.set_torque_enable(dxl_id, self.TORQUE_DISABLE, locked=True)
            
            # Write P, I, and D gains (Note: These are 2-byte values)
            result, error = self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, self.ADDR_POSITION_P_GAIN, p_gain)
            self._check_comm(result, error, f"Setting P-Gain for ID {dxl_id}")
            
            result, error = self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, self.ADDR_POSITION_I_GAIN, i_gain)
            self._check_comm(result, error, f"Setting I-Gain for ID {dxl_id}")
            
            result, error = self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, self.ADDR_POSITION_D_GAIN, d_gain)
            self._check_comm(result, error, f"Setting D-Gain for ID {dxl_id}")
            
            self.set_torque_enable(dxl_id, self.TORQUE_ENABLE, locked=True)
            rospy.loginfo(f"Set PID gains for ID {dxl_id} to P:{p_gain}, I:{i_gain}, D:{d_gain}")
            
        finally:
            if not locked: self.comm_lock.release()


    def set_operating_mode(self, dxl_id, mode, locked=False):
        if self.current_modes.get(dxl_id) == mode: return
        if not locked: self.comm_lock.acquire()
        try:
            self.set_torque_enable(dxl_id, self.TORQUE_DISABLE, locked=True)
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_OPERATING_MODE, mode)
            self._check_comm(result, error, f"Setting operating mode for ID {dxl_id}")
            if result == COMM_SUCCESS and error == 0: self.current_modes[dxl_id] = mode
            self.set_pid_gains(dxl_id, 1000, 100, 100, locked=True)  # Example PID gains

            self.set_torque_enable(dxl_id, self.TORQUE_ENABLE, locked=True)
        finally:
            if not locked: self.comm_lock.release()

    def write_positions(self, dxl_ids_to_move, target_positions):
        with self.comm_lock:
            for i, dxl_id in enumerate(dxl_ids_to_move):
                if self.current_modes.get(dxl_id) != self.OPERATING_MODE_POSITION: continue
                goal_pos = target_positions[i]
                result, error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.ADDR_GOAL_POSITION, goal_pos)
                self._check_comm(result, error, f"Writing position for ID {dxl_id}")

    def write_pwms(self, dxl_ids_to_move, target_pwms):
        with self.comm_lock:
            for i, dxl_id in enumerate(dxl_ids_to_move):
                if self.current_modes.get(dxl_id) != self.OPERATING_MODE_PWM: continue
                goal_pwm = target_pwms[i]
                result, error = self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, self.ADDR_GOAL_PWM, goal_pwm)
                self._check_comm(result, error, f"Writing PWM for ID {dxl_id}")

    def read_position(self, dxl_id):
        with self.comm_lock:
            if not self.port_is_open: return 0
            present_pos, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_POSITION)
            self._check_comm(result, error, f"Reading position for ID {dxl_id}")
            return present_pos if result == COMM_SUCCESS else 0

    def read_positions(self):
        return [self.read_position(dxl_id) for dxl_id in self.dxl_ids]
        
    def read_load(self, dxl_id):
        with self.comm_lock:
            if not self.port_is_open: return 0
            present_load, result, error = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_LOAD)
            self._check_comm(result, error, f"Reading load for ID {dxl_id}")
            if result != COMM_SUCCESS: return 0
            return present_load - 65536 if present_load > 32767 else present_load

    def wait_until_reached(self, dxl_ids_to_wait, target_positions):
        for i, dxl_id in enumerate(dxl_ids_to_wait):
            goal_pos = target_positions[i]
            while not rospy.is_shutdown():
                if abs(goal_pos - self.read_position(dxl_id)) <= self.MOVING_THRESHOLD_POSITION: break
                rospy.sleep(0.02)

    def wait_until_load_threshold(self, dxl_id, threshold):
        rospy.loginfo(f"Waiting for ID {dxl_id} load to be >= {abs(threshold)}...")
        while not rospy.is_shutdown():
            if abs(self.read_load(dxl_id)) >= abs(threshold):
                rospy.loginfo(f"ID {dxl_id} load threshold reached.")
                break
            rospy.sleep(0.02)

    def move_z_up(self):
        rospy.loginfo(f"Moving Z-axis (ID: {self.z_axis_id}) UP to {self.z_up_position}")
        self.set_operating_mode(self.z_axis_id, self.OPERATING_MODE_POSITION)
        self.write_positions([self.z_axis_id], [self.z_up_position])
        self.wait_until_reached([self.z_axis_id], [self.z_up_position])
        rospy.loginfo(f"Z-axis is UP.")

    def move_z_down(self):
        rospy.loginfo(f"Moving Z-axis (ID: {self.z_axis_id}) DOWN with PWM {self.z_down_pwm}")
        self.set_operating_mode(self.z_axis_id, self.OPERATING_MODE_PWM)
        self.write_pwms([self.z_axis_id], [self.z_down_pwm])
        self.wait_until_load_threshold(self.z_axis_id, self.STOPPED_LOAD_THRESHOLD)
        self.write_pwms([self.z_axis_id], [0]) # Stop the motor
        rospy.loginfo("Z-axis is DOWN (stopped by load).")

    def disable_torque_and_close(self):
        with self.comm_lock:
            rospy.loginfo("Disabling torque for all motors.")
            for dxl_id in self.dxl_ids:
                self.set_torque_enable(dxl_id, self.TORQUE_DISABLE, locked=True)
            if self.port_is_open:
                self.portHandler.closePort()
                self.port_is_open = False
                rospy.loginfo("Port closed.")

# ==============================================================================
# SECTION 3: UTILITY AND KINEMATICS
# (DXF parsing function is now fixed)
# ==============================================================================

def plot_trajectories(trajectories, offset=[0,0]):
    if not MATPLOTLIB_AVAILABLE:
        rospy.logwarn("Matplotlib not found. Skipping trajectory plot.")
        return
    plt.figure(figsize=(10, 8))
    ax = plt.gca()
    for i, traj in enumerate(trajectories):
        offset_traj = traj + np.array(offset)
        ax.plot(offset_traj[:, 0], offset_traj[:, 1], marker='.')
    ax.set_aspect('equal', adjustable='box')
    plt.title(f'DXF Trajectories Preview (Offset by {np.round(offset, 3)})')
    plt.xlabel('X coordinate (m)')
    plt.ylabel('Y coordinate (m)')
    plt.grid(True)
    plt.show(block=False)
    plt.pause(2)
    rospy.loginfo("Trajectory plot displayed. Continuing execution...")

def extract_trajectories_from_dxf_in_meters(filepath, curve_sampling_tolerance=0.001, straight_segment_max_length_m=0.001):
    """
    Extracts trajectories from a DXF file, including more entity types, and normalizes their positions.
    This version also adds more points to straight line segments.

    This function reads a DXF file, finds various drawing entities (LWPOLYLINE, POLYLINE,
    LINE, ARC, CIRCLE, ELLIPSE, SPLINE, and explodes INSERT entities),
    and converts their coordinates from millimeters to meters. It samples curved
    entities to generate a sequence of points and interpolates points along
    straight line segments to ensure a minimum point density.
    It then takes the starting point of the *first* trajectory found and uses it
    as a global origin. All trajectories are then translated by this same offset,
    ensuring the first trajectory starts at (0,0) and all others maintain their
    relative position to it.

    Args:
        filepath (str): The path to the DXF file.
        curve_sampling_tolerance (float): A tolerance value for sampling curved
                                          entities (e.g., arcs, circles, splines).
                                          Smaller values result in more points.
                                          Value in original DXF drawing units (mm).
        straight_segment_max_length_m (float): The maximum desired length in meters
                                               between points on a straight line segment.
                                               If a segment is longer, points will be
                                               interpolated.

    Returns:
        list: A list of numpy arrays, where each array represents a trajectory
              normalized relative to the start of the first trajectory.
              Returns an empty list if no trajectories are found or an error occurs.
    """
    if not EZDXF_AVAILABLE:
        rospy.logerr("ezdxf library not available. Cannot extract trajectories from DXF.")
        return []

    raw_trajectories = []

    # Helper function to interpolate points on a straight line segment
    def interpolate_segment(p1_meters, p2_meters, max_length_m):
        segment_points = [p1_meters]
        p1 = np.array(p1_meters)
        p2 = np.array(p2_meters)
        
        distance = np.linalg.norm(p2 - p1)
        if distance > max_length_m:
            num_segments = int(np.ceil(distance / max_length_m))
            for i in range(1, num_segments):
                ratio = float(i) / num_segments
                interpolated_point = p1 + ratio * (p2 - p1)
                segment_points.append(interpolated_point.tolist())
        segment_points.append(p2_meters)
        return segment_points

    try:
        doc = ezdxf.readfile(filepath)
        msp = doc.modelspace()

        exploded_entities = []
        for entity in msp:
            if entity.dxftype() == 'INSERT':
                exploded_entities.extend(list(entity.explode()))
            else:
                exploded_entities.append(entity)

        for entity in exploded_entities:
            current_entity_points = []
            dxftype = entity.dxftype()

            if dxftype == 'LWPOLYLINE':
                # get_points can return a list of (x, y, start_width, end_width, bulge)
                # or just (x, y) if format='xy'. We need to process segments.
                # If curve_accuracy is supported, use it. Otherwise, rely on default.
                try:
                    raw_points_mm = entity.get_points(format='xy', curve_accuracy=curve_sampling_tolerance)
                except TypeError: # Older ezdxf versions don't have curve_accuracy
                    raw_points_mm = entity.get_points(format='xy')
                
                if not raw_points_mm:
                    continue

                # Convert all points to meters first
                points_in_meters = [[p[0] / 1000.0, p[1] / 1000.0] for p in raw_points_mm]
                
                # Iterate through segments for interpolation
                current_entity_points.append(points_in_meters[0]) # Add first point
                for i in range(len(points_in_meters) - 1):
                    p1 = points_in_meters[i]
                    p2 = points_in_meters[i+1]
                    interpolated_seg_points = interpolate_segment(p1, p2, straight_segment_max_length_m)
                    # Add all interpolated points except the very first one (which is p1, already added or is part of prev segment's end)
                    current_entity_points.extend(interpolated_seg_points[1:])


            elif dxftype == 'POLYLINE':
                # Similar to LWPOLYLINE, convert vertices to meters and then interpolate
                raw_vertices = [vertex.dxf.location for vertex in entity.vertices]
                if not raw_vertices:
                    continue

                points_in_meters = [[v.x / 1000.0, v.y / 1000.0] for v in raw_vertices]

                current_entity_points.append(points_in_meters[0]) # Add first point
                for i in range(len(points_in_meters) - 1):
                    p1 = points_in_meters[i]
                    p2 = points_in_meters[i+1]
                    interpolated_seg_points = interpolate_segment(p1, p2, straight_segment_max_length_m)
                    current_entity_points.extend(interpolated_seg_points[1:])


            elif dxftype == 'LINE':
                start_m = [entity.dxf.start.x / 1000.0, entity.dxf.start.y / 1000.0]
                end_m = [entity.dxf.end.x / 1000.0, entity.dxf.end.y / 1000.0]
                current_entity_points.extend(interpolate_segment(start_m, end_m, straight_segment_max_length_m))

            elif dxftype == 'ARC':
                # Use flattening, it handles sampling based on tolerance
                for p in entity.flattening(curve_sampling_tolerance):
                    current_entity_points.append([p.x / 1000.0, p.y / 1000.0])

            elif dxftype == 'CIRCLE':
                # Use flattening, it handles sampling based on tolerance
                for p in entity.flattening(curve_sampling_tolerance):
                    current_entity_points.append([p.x / 1000.0, p.y / 1000.0])

            elif dxftype == 'ELLIPSE':
                # Use flattening, it handles sampling based on tolerance
                for p in entity.flattening(curve_sampling_tolerance):
                    current_entity_points.append([p.x / 1000.0, p.y / 1000.0])

            elif dxftype == 'SPLINE':
                # Use flattening, it handles sampling based on tolerance
                for p in entity.flattening(curve_sampling_tolerance):
                    current_entity_points.append([p.x / 1000.0, p.y / 1000.0])

            if current_entity_points:
                # Filter out duplicate consecutive points
                if len(current_entity_points) > 1:
                    unique_points = [current_entity_points[0]]
                    for i in range(1, len(current_entity_points)):
                        if not np.allclose(current_entity_points[i], unique_points[-1]): # Compare to last unique point
                            unique_points.append(current_entity_points[i])
                    current_entity_points = unique_points
                
                if len(current_entity_points) > 0: # Ensure we still have points after de-duplication
                    raw_trajectories.append(np.array(current_entity_points))

    except IOError:
        rospy.logerr(f"Error: Could not find or open the DXF file at '{filepath}'")
        return []
    except ezdxf.DXFStructureError:
        rospy.logerr(f"Error: The DXF file at '{filepath}' is invalid or corrupt.")
        return []
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred while processing DXF: {e}")
        return []

    if not raw_trajectories:
        rospy.logwarn(f"No valid trajectories found in {filepath}.")
        return []

    # Ensure the first trajectory actually has points before trying to access its first element
    if not raw_trajectories[0].size > 0:
        rospy.logerr(f"First trajectory found is empty in {filepath}. Cannot normalize.")
        return []

    global_offset = raw_trajectories[0][0]
    normalized_trajectories = [traj - global_offset for traj in raw_trajectories]

    rospy.loginfo(f"Successfully extracted and normalized {len(normalized_trajectories)} trajectories from {filepath}.")
    return normalized_trajectories

# --- Robot Kinematics and Joint Conversions (Unchanged) ---
DXL_IDS = [1, 2, 3]
POSITION_LIMITS = [(1100, 2900), (1100, 3000), (2000, 3270)]
PRISMATIC_MIN_M, PRISMATIC_MAX_M = 0.0, 0.05

def rad_to_dxl_units(val, joint_index):
    dxl_min, dxl_max = POSITION_LIMITS[joint_index]
    if joint_index == 2:
        norm_val = (val - PRISMATIC_MIN_M) / (PRISMATIC_MAX_M - PRISMATIC_MIN_M)
        dxl_val = dxl_min + norm_val * (dxl_max - dxl_min)
    else:
        if joint_index == 1: val -= (np.pi / 2.0)
        dxl_val = 2048 + val * (2048 / np.pi)
    return np.clip(int(dxl_val), dxl_min, dxl_max)

def dxl_units_to_rad(dxl_val, joint_index):
    dxl_min, dxl_max = POSITION_LIMITS[joint_index]
    if joint_index == 2:
        if (dxl_max - dxl_min) == 0: return PRISMATIC_MIN_M
        norm_val = (dxl_val - dxl_min) / float(dxl_max - dxl_min)
        return PRISMATIC_MIN_M + norm_val * (PRISMATIC_MAX_M - PRISMATIC_MIN_M)
    else:
        rad = (dxl_val - 2048) * (np.pi / 2048.0)
        if joint_index == 1: rad += (np.pi / 2.0)
        return rad



def ik_2dof_with_dual_offsets_and_limits(L1, L1_offset, L2, L2_offset, x, y):
    """
    Calculates inverse kinematics for a 2-DOF planar arm with fixed perpendicular
    offsets on both links.

    This function models the arm by finding the intersection of two circles to
    determine the position of the second joint (the wrist), which accommodates
    offsets on both links.

    Args:
        L1 (float): Length of link 1.
        L1_offset (float): Perpendicular offset on link 1. The value is -31mm.
        L2 (float): Primary length of link 2.
        L2_offset (float): Perpendicular offset on link 2.
        x (float): Target x-coordinate in the frame of joint 1.
        y (float): Target y-coordinate in the frame of joint 1.

    Returns:
        tuple[float, float] | None: A tuple of (theta1_rad, theta2_rad) if a
                                     solution is found within limits, otherwise None.
    """
    # --- 1. Calculate effective lengths and angle offsets for both links ---
    L_eff_1 = math.sqrt(L1**2 + L1_offset**2)
    phi_offset_1 = math.atan2(L1_offset, L1)

    L_eff_2 = math.sqrt(L2**2 + L2_offset**2)
    phi_offset_2 = math.atan2(L2_offset, L2)

    # --- 2. Check for reachability ---
    # Distance from the origin (joint 1) to the target point (x, y)
    d = math.sqrt(x**2 + y**2)

    # The arm can't reach if the target is further than the sum of effective lengths
    # or closer than the difference of effective lengths.
    if d > L_eff_1 + L_eff_2 or d < abs(L_eff_1 - L_eff_2):
        rospy.logwarn("IK: Target is out of reachable range.")
        return None
    
    # --- 3. Solve for the wrist position using circle-circle intersection ---
    # 'a' is the distance from joint 1 to the midpoint of the two intersection points
    # 'h' is the distance from that midpoint to each intersection point
    try:
        a = (L_eff_1**2 - L_eff_2**2 + d**2) / (2 * d)
        h = math.sqrt(L_eff_1**2 - a**2)
    except ValueError:
        # Catches floating point errors where sqrt argument is slightly negative
        rospy.logwarn("IK: Cannot compute wrist position (sqrt of negative number).")
        return None

    # Coordinates of the midpoint P2
    x2 = x * a / d
    y2 = y * a / d

    # --- 4. Calculate joint angles for both possible solutions ---
    # Solution 1 (e.g., "elbow down")
    wrist_x1 = x2 + h * y / d
    wrist_y1 = y2 - h * x / d
    
    # Solution 2 (e.g., "elbow up")
    wrist_x2 = x2 - h * y / d
    wrist_y2 = y2 + h * x / d

    solutions = [(wrist_x1, wrist_y1), (wrist_x2, wrist_y2)]
    urdf_limit = np.pi / 2

    for wx, wy in solutions:
        # Global angle of the effective first link
        theta1_eff = math.atan2(wy, wx)
        # Correct for the physical link's angle offset
        theta1_rad = theta1_eff - phi_offset_1

        # Global angle of the effective second link
        theta2_eff_global = math.atan2(y - wy, x - wx)
        # Relative angle of the effective second link w.r.t. the effective first link
        theta2_eff_relative = theta2_eff_global - theta1_eff
        # Correct for the physical link's angle offset
        theta2_rad = theta2_eff_relative - phi_offset_2
        
        # Normalize angles to be within [-pi, pi] for consistent checks
        theta1_rad = (theta1_rad + np.pi) % (2 * np.pi) - np.pi
        theta2_rad = (theta2_rad + np.pi) % (2 * np.pi) - np.pi

        # Check if the calculated angles are within the specified URDF limits
        if -urdf_limit <= theta1_rad <= urdf_limit and -urdf_limit <= theta2_rad <= urdf_limit:
            return (theta1_rad, theta2_rad)

    rospy.logwarn("IK: No solution found within URDF limits.")
    return None


class RobotKinematics:
    def __init__(self):
        self.L1 = 0.230
        self.L1_offset = -0.031 # The -31mm offset for L1
        self.L2 = 0.2056
        self.L2_offset = 0.038  # The 38mm x-offset for L2
        self.joint1_offset_xy = np.array([0.0011, 0.07051])
        self.current_end_effector_xy_base_frame = np.array([0.0, 0.0])

    def get_target_relative_to_joint1(self, target_xy_base_frame):
        """Translates a target in the base frame to the robot's joint 1 frame."""
        return target_xy_base_frame - self.joint1_offset_xy

    def forward_kinematics_2d(self, q1_rad, q2_rad):
        """
        Calculates end-effector position from joint angles, considering offsets
        on both links.
        """
        # Position of joint 2 (wrist), considering L1 and its offset
        # This is a 2D rotation of the point (L1, L1_offset) by q1
        wrist_x = self.L1 * math.cos(q1_rad) - self.L1_offset * math.sin(q1_rad)
        wrist_y = self.L1 * math.sin(q1_rad) + self.L1_offset * math.cos(q1_rad)

        # Position of end-effector relative to joint 2 (wrist)
        # This is a 2D rotation of the point (L2, L2_offset) by (q1 + q2)
        end_eff_rel_x = self.L2 * math.cos(q1_rad + q2_rad) - self.L2_offset * math.sin(q1_rad + q2_rad)
        end_eff_rel_y = self.L2 * math.sin(q1_rad + q2_rad) + self.L2_offset * math.cos(q1_rad + q2_rad)
        
        # Total position in the joint 1 frame is the sum of the two vectors
        x_rel = wrist_x + end_eff_rel_x
        y_rel = wrist_y + end_eff_rel_y

        # Add the base offset to get the position in the base frame
        return np.array([x_rel, y_rel]) + self.joint1_offset_xy

    def set_initial_end_effector_pos(self, initial_q1, initial_q2):
        self.current_end_effector_xy_base_frame = self.forward_kinematics_2d(initial_q1, initial_q2)
        rospy.loginfo(f"Initial end-effector XY (base frame): {self.current_end_effector_xy_base_frame}")


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
        DXF_PATH = "./Not_so_much_pain_afterall.dxf"

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
            ik_sol = ik_2dof_with_dual_offsets_and_limits(robot_kinematics.L1,-0.031, robot_kinematics.L2,0.038, start_point_rel_j1[0], start_point_rel_j1[1])

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
                point_rel_j1 = robot_kinematics.get_target_relative_to_joint1(point_world)
                ik_sol = ik_2dof_with_dual_offsets_and_limits(robot_kinematics.L1, -0.031, robot_kinematics.L2, 0.038, point_rel_j1[0], point_rel_j1[1])
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