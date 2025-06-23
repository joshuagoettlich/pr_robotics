#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from dynamixel_sdk import *
import pinocchio as pin
import sys
import tty
import termios
import matplotlib.pyplot as plt
from controll_class import *

EZDXF_AVAILABLE = False
try:
    import ezdxf
    EZDXF_AVAILABLE = True
except ImportError:
    print("The 'ezdxf' library is required to read trajectories from DXF files.")
    print("Please install it using: pip install ezdxf")
MATPLOTLIB_AVAILABLE = False
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    print("The 'matplotlib' library is recommended for plotting trajectories.")
    print("Please install it using: pip install matplotlib")

# === Utility to read single keypress ===
def getch():
    """Gets a single character from standard input. Does not echo to the screen."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def plot_trajectories(trajectories, offset=[0,0]):
    """
    Plots the extracted trajectories using Matplotlib for a visual preview.

    Args:
        trajectories (list): A list of numpy arrays, where each array is a trajectory.
        offset (list, optional): An [x, y] offset to apply to all trajectories
                                 before plotting. Defaults to [0,0].
    """
    if not MATPLOTLIB_AVAILABLE:
        print("Matplotlib not found. Skipping trajectory plot.")
        return

    # --- START: New/Modified code ---
    # Convert the offset to a numpy array for vectorized addition.
    offset_array = np.array(offset)
    # --- END: New/Modified code ---

    plt.figure(figsize=(10, 8)) # Increased figure size for better viewing
    ax = plt.gca()
    
    for i, traj in enumerate(trajectories):
        # --- START: New/Modified code ---
        # Add the offset to every point in the current trajectory.
        # NumPy broadcasting handles this efficiently.
        offset_traj = traj + offset_array
        # --- END: New/Modified code ---

        # Plot each trajectory with a different color using the offset data
        ax.plot(offset_traj[:, 0], offset_traj[:, 1], marker='.', label=f'Trajectory {i+1}')
        # Mark the start point in green, using the offset data
        ax.plot(offset_traj[0, 0], offset_traj[0, 1], 'go', markersize=10, label=f'_Start {i+1}')
        # Mark the end point in red, using the offset data
        ax.plot(offset_traj[-1, 0], offset_traj[-1, 1], 'ro', markersize=10, label=f'_End {i+1}')

    ax.set_aspect('equal', adjustable='box')
    plt.title('Extracted DXF Trajectories (Preview)')
    plt.xlabel(f'X coordinate (m) [Offset: {offset[0]}]')
    plt.ylabel(f'Y coordinate (m) [Offset: {offset[1]}]')
    plt.grid(True)
    
    print("Displaying trajectory plot. Close the plot window to continue.")
    plt.show()  # This will pause the script until the plot window is closed

def extract_trajectories_from_dxf_in_meters(filepath):
    """
    Extracts trajectories from a DXF file and normalizes their positions.

    This function reads a DXF file, finds all LWPOLYLINE and POLYLINE entities,
    and converts their coordinates from millimeters to meters. It then takes the
    starting point of the *first* trajectory found and uses it as a global origin.
    All trajectories are then translated by this same offset, ensuring the first
e    trajectory starts at (0,0) and all others maintain their relative position
    to it.
    
    Args:
        filepath (str): The path to the DXF file.

    Returns:
        list: A list of numpy arrays, where each array represents a trajectory
              normalized relative to the start of the first trajectory.
              Returns an empty list if no trajectories are found or an error occurs.
    """
    raw_trajectories = []
    try:
        doc = ezdxf.readfile(filepath)
        msp = doc.modelspace()
        
        # --- Step 1: Extract all trajectories without normalization ---
        # Iterate through entities to better preserve the order in the DXF file.
        for entity in msp:
            points = []
            if entity.dxftype() == 'LWPOLYLINE':
                # The get_points() method returns points as (x, y) tuples
                for point in entity.get_points(format='xy'):
                    point_in_meters = [point[0] / 1000.0, point[1] / 1000.0]
                    points.append(point_in_meters)
            
            elif entity.dxftype() == 'POLYLINE':
                # Ensure the polyline is a 2D polyline
                if not entity.is_3d_polyline and not entity.is_closed:
                    for vertex in entity.vertices:
                        point_in_meters = [vertex.dxf.location.x / 1000.0, vertex.dxf.location.y / 1000.0]
                        points.append(point_in_meters)

            if points:
                raw_trajectories.append(np.array(points))

    except IOError:
        print(f"Error: Could not find or open the DXF file at '{filepath}'")
        return []
    except ezdxf.DXFStructureError:
        print(f"Error: The DXF file at '{filepath}' is invalid or corrupt.")
        return []

    # --- Step 2: Normalize all trajectories based on the first one ---
    if not raw_trajectories:
        print(f"No valid trajectories found in {filepath}.")
        return []

    # Get the first point of the very first trajectory to use as the global offset.
    global_offset = raw_trajectories[0][0]
    
    # Apply this single offset to all trajectories.
    normalized_trajectories = [traj - global_offset for traj in raw_trajectories]
    
    print(f"Successfully extracted and normalized {len(normalized_trajectories)} trajectories from {filepath}.")
    return normalized_trajectories

# === Dynamixel Controller ===

# === Pinocchio Robot Wrapper for FK/IK (Corrected) ===
class RobotPinocchio:
    """Handles kinematics and dynamics using Pinocchio, assuming a full robot model."""
    def __init__(self, urdf_path, base_link, end_link):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.base_link = base_link
        self.end_link = end_link
        self.end_link_frame_id = self.model.getFrameId(self.end_link)

        # Store the joint limits from the URDF model
        self.lower_limits = self.model.lowerPositionLimit
        self.upper_limits = self.model.upperPositionLimit
        print("Robot joint limits loaded from URDF:")
        print(f"Lower: {self.lower_limits}")
        print(f"Upper: {self.upper_limits}")

    def forward_kinematics(self, q):
        # q is now assumed to be the full configuration vector
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.end_link_frame_id].translation

    def inverse_kinematics(self, target_pos, q_seed, max_iter=10000, tol=1e-3, alpha_initial=1.0, lambda_damping=1e-4):
        """
        Computes IK for a given target, respecting joint limits, using Damped Least Squares.
        """
        q = q_seed.copy()
        print(f"Initial joint angles (seed): {q}")
        print(f"Target position for IK: {target_pos}")
        target_dim = len(target_pos)

        for i in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current_pos_full = self.data.oMf[self.end_link_frame_id].translation
            error = target_pos - current_pos_full[:target_dim]

            if np.linalg.norm(error) < tol:
                print(f"Solution found in {i+1} iterations!")
                return q

            # Compute the Jacobian in the WORLD frame
            J = pin.computeFrameJacobian(self.model, self.data, q, self.end_link_frame_id, pin.ReferenceFrame.WORLD)
            J_pos = J[:target_dim, :]

            # Damped Least Squares
            JTJ = J_pos.T @ J_pos
            damp = lambda_damping**2 * np.identity(J_pos.shape[1])
            
            try:
                # Solve for the change in ALL joint angles
                dq = np.linalg.solve(JTJ + damp, J_pos.T @ error)
            except np.linalg.LinAlgError:
                print("Linear algebra error, could not solve for dq.")
                return None

            # Line search for a valid step
            alpha = alpha_initial
            for _ in range(10):
                q_next = q + alpha * dq
                if np.all(q_next >= self.lower_limits) and np.all(q_next <= self.upper_limits):
                    q = q_next
                    break
                alpha /= 2.0
            else:
                # If line search fails, we might be stuck. Clipping as a last resort.
                q += alpha_initial * dq
                q = np.clip(q, self.lower_limits, self.upper_limits)


        rospy.logwarn("IK failed to converge within max iterations.")
        return None

# === Global Conversion Functions ===
# These need to be global so they can be defined once with the limits.
# Define motor IDs and position limits globally for the conversion functions.
DXL_IDS = [1, 2, 3]
POSITION_LIMITS = [(1100, 2900), (1100, 3000), (2800, 3100)] # Min/Max for each motor

def rad_to_dxl_units(rad, joint_index):
    # This conversion maps a -pi to pi range to the dynamixel limits.
    # YOU MUST ADJUST THIS to your robot's true mechanical limits in radians.
    rad=rad + np.pi  # Adjusting the radian value to match the robot's configuration
    dxl_min, dxl_max = POSITION_LIMITS[joint_index]
    # Clip the value to be within the safe operating range
    return np.clip(int(rad*2048/np.pi), dxl_min, dxl_max)

def dxl_units_to_rad(dxl_val, joint_index):
    dxl_val= dxl_val - 2048  # Adjusting the dynamixel value to match the robot's configuration
    return dxl_val * (np.pi / 2048.0)  # Assuming the dynamixel units are in degrees

# === ROS Interface for Dynamixel ===
class DynamixelRosInterface:
    """Handles ROS communication and orchestrates the dynamixel controller."""
    def __init__(self, dxl_ids, position_limits):
        self.dxl_controller = DynamixelController(dxl_ids, position_limits)
        self.joint_names = [f"joint{i+1}" for i in range(len(dxl_ids))]

        rospy.init_node('dynamixel_controller_node', anonymous=True)
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.joint_names

    def publish_joint_positions(self, positions_dxl_units):
        self.dxl_controller.write_positions(positions_dxl_units)
        
        # Convert positions to radians for ROS message
        radians = [dxl_units_to_rad(pos, i) for i, pos in enumerate(positions_dxl_units)]
        self.joint_state_msg.position = radians
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.joint_state_msg)

    def get_current_joint_positions_rad(self):
        dxl_positions = self.dxl_controller.read_positions()
        # Convert to radians
        return np.array([dxl_units_to_rad(pos, i) for i, pos in enumerate(dxl_positions)])

    def shutdown(self):
        self.dxl_controller.disable_torque_and_close()

    def wait_until_reached(self, target_positions):
        """
        Waits until the motors reach the target positions.
        This function assumes target_positions is in Dynamixel units.
        """
        self.dxl_controller.wait_until_reached(target_positions)

# === Main Execution Block ===
if __name__ == "__main__":#
    try:
        # --- Robot Configuration ---
        z_joint_index = 2 # The 3rd motor (index 2) controls the Z-axis
        
        # Define Z positions in Dynamixel units for quick access
        z_min_dxl, z_max_dxl = POSITION_LIMITS[z_joint_index]
        z_up_dxl = z_max_dxl
        z_down_dxl = z_min_dxl

        # Define Z positions in RADIANS for IK seeding and state
        z_up_rad = dxl_units_to_rad(z_up_dxl, z_joint_index)
        z_down_rad = dxl_units_to_rad(z_down_dxl, z_joint_index)
        
        # --- Pinocchio Robot Model Setup ---
        urdf_path = "./urdf/robot.urdf"
        base_link = "base_link"
        end_link = "end_effector_link"

        robot = RobotPinocchio(urdf_path, base_link, end_link)
        ros_interface = DynamixelRosInterface(DXL_IDS, POSITION_LIMITS)

        rospy.sleep(1.0)  # Wait for ROS publishers and subscribers to connect

        # --- Trajectory Definition ---
        # Load trajectories from the specified DXF file
        dxf_path = "./Not_so_much_pain_afterall.dxf" # <<<< SET YOUR DXF FILE PATH HERE
        aligned_trajectories = extract_trajectories_from_dxf_in_meters(dxf_path)
        
        if not aligned_trajectories:
            print("No trajectories found in the DXF file. Exiting.")
            sys.exit(0)
        # Get the initial state of the robot in radians
        q_current = ros_interface.get_current_joint_positions_rad()
        current_dxl_pos = [rad_to_dxl_units(rad, j) for j, rad in enumerate(q_current)]
        print(f"Initial joint positions (radians): {q_current}")
        print(f"Initial joint positions (Dynamixel units): {current_dxl_pos}")
        current_pos= robot.forward_kinematics(q_current)[0:2]  # Get the 2D position of the end effector
        print(f"Initial end effector position: {current_pos}")

        print("Starting trajectory following. Press Ctrl+C to stop.")

        # --- Plot the trajectories before execution ---
        plot_trajectories(aligned_trajectories)

        rate = rospy.Rate(20) # Control loop frequency in Hz

        
        for traj in aligned_trajectories:
            # Lift the pen before starting a new trajectory
            print("Lifting pen to start new trajectory...")
            current_dxl_pos = [rad_to_dxl_units(rad, j) for j, rad in enumerate(q_current)]
            current_dxl_pos[z_joint_index] = z_up_dxl
            ros_interface.publish_joint_positions(current_dxl_pos)
            rospy.sleep(0.5) # Give it time to lift
            
            # Update current state after lifting
            q_current = ros_interface.get_current_joint_positions_rad()

            for i, point in enumerate(traj):
                if rospy.is_shutdown():
                    break
                
                point = np.array(point)+ current_pos  # Adjust point to the current position
                print(f"Moving to point {i+1}/{len(traj)}: {point}")

                # --- Run Inverse Kinematics ---
                # The target is the 2D point, but the seed is the FULL 3D joint state.
                q_ik_full_rad = robot.inverse_kinematics(point, q_seed=q_current)
                
                if q_ik_full_rad is not None:
                    # The IK result is a 3-element radian vector.
                    # We override the z-joint value to ensure the pen is down.
                    q_target_rad = q_ik_full_rad
                    q_target_rad[z_joint_index] = z_down_rad

                    # Update q_current for the next iteration's seed
                    q_current = q_target_rad

                    # Convert the final target radians to dynamixel units for all motors
                    print(f"Target joint positions (radians): {q_target_rad}")
                    print(f"Target joint positions (Dynamixel units): {[rad_to_dxl_units(rad, j) for j, rad in enumerate(q_target_rad)]}")
                    q_target_dxl = [rad_to_dxl_units(rad, j) for j, rad in enumerate(q_target_rad)]
                    ros_interface.publish_joint_positions(q_target_dxl)
                    ros_interface.wait_until_reached(q_target_dxl)

                else:
                    rospy.logwarn(f"IK failed for point {point}, skipping.")

                rate.sleep()

        print("All trajectories finished.")

    except rospy.ROSInterruptException:
        print("ROS node interrupted.")
    except (IOError, termios.error):
        print("Input/output error, possibly due to remote connection. Shutting down.")
    finally:
        # Gracefully shut down the motors and close the port
        if 'ros_interface' in locals():
            ros_interface.shutdown()