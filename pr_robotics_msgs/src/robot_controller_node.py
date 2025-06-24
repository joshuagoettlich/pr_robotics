#!/usr/bin/env python3

import rospy
import math
import numpy as np
import sys
import tty # For getch, but it's not used in this node directly
import termios # For getch, but it's not used in this node directly

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Point # For initial EE pose

# Import your DynamixelController directly
# This assumes controll_class.py is in the same directory or accessible via PYTHONPATH
from controll_class import DynamixelController

# === Global Conversion Functions (shared constants) ===
# Define motor IDs and position limits globally for the conversion functions.
# These MUST match the values in ik_solver_node and any physical setup.
DXL_IDS = [1, 2, 3] # Assuming your robot has 3 Dynamixel motors
POSITION_LIMITS = [(1100, 2900), (1100, 3000), (2800, 3100)] # Min/Max DXL units per motor

# For prismatic joint (joint3, index 2), map its DXL units to meters.
PRISMATIC_MIN_M = 0.0  # meters for the prismatic joint at its lowest DXL unit limit
PRISMATIC_MAX_M = 0.05 # meters for the prismatic joint at its highest DXL unit limit

# Store a global reference for the current prismatic position in meters/radians
# This is mainly for internal state tracking and publishing, not direct control input.
_current_q3_prismatic_val = PRISMATIC_MIN_M # Initialize to a safe default

def rad_to_dxl_units(val, joint_index):
    """
    Converts a joint angle from radians (for revolute) or position from meters (for prismatic)
    to Dynamixel units. This mapping MUST be correct for your specific Dynamixel setup.
    """
    dxl_min, dxl_max = POSITION_LIMITS[joint_index]

    if joint_index == 2: # Prismatic joint (joint3)
        if PRISMATIC_MAX_M - PRISMATIC_MIN_M == 0:
            rospy.logwarn("Prismatic joint max and min meters are the same, cannot convert.")
            return dxl_min
        normalized_val = (val - PRISMATIC_MIN_M) / (PRISMATIC_MAX_M - PRISMATIC_MIN_M)
        dxl_val = dxl_min + normalized_val * (dxl_max - dxl_min)
        return np.clip(int(dxl_val), dxl_min, dxl_max)
    else: # Revolute joints (joint1, joint2)
        dxl_val = int(2048 + val * (2048 / np.pi))
        return np.clip(dxl_val, dxl_min, dxl_max)

def dxl_units_to_rad(dxl_val, joint_index):
    """
    Converts a joint position from Dynamixel units to radians (for revolute) or meters (for prismatic).
    """
    dxl_min, dxl_max = POSITION_LIMITS[joint_index]

    if joint_index == 2: # Prismatic joint (joint3)
        if dxl_max - dxl_min == 0:
            rospy.logwarn("Prismatic joint max and min DXL units are the same, cannot convert.")
            return PRISMATIC_MIN_M
        normalized_val = (dxl_val - dxl_min) / (dxl_max - dxl_min)
        val = PRISMATIC_MIN_M + normalized_val * (PRISMATIC_MAX_M - PRISMATIC_MIN_M)
        return val
    else: # Revolute joints (joint1, joint2)
        rad = (dxl_val - 2048) * (np.pi / 2048.0)
        return rad

# === Robot Kinematics (FK for initial pose) ===
class RobotKinematics:
    """Manages robot specific kinematics for FK."""
    def __init__(self):
        self.L1 = 0.230
        self.L2 = 0.2056
        self.joint1_offset_xy = np.array([0.0011, 0.07051])

    def forward_kinematics_2d(self, q1_rad: float, q2_rad: float) -> np.ndarray:
        x_rel_j1 = self.L1 * math.cos(q1_rad) + self.L2 * math.cos(q1_rad + q2_rad)
        y_rel_j1 = self.L1 * math.sin(q1_rad) + self.L2 * math.sin(q1_rad + q2_rad)
        x_base_frame = x_rel_j1 + self.joint1_offset_xy[0]
        y_base_frame = y_rel_j1 + self.joint1_offset_xy[1]
        return np.array([x_base_frame, y_base_frame])

class RobotControllerNode:
    def __init__(self):
        rospy.init_node('robot_controller_node', anonymous=True)

        self.dxl_controller = DynamixelController(DXL_IDS, POSITION_LIMITS)
        self.robot_kinematics = RobotKinematics()

        self.joint_names = ['joint1', 'joint2', 'joint3'] # Match URDF names

        # Publishers
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.initial_ee_pose_pub = rospy.Publisher('/robot_initial_ee_pose', Point, queue_size=1, latch=True) # Latch to ensure DXF reader gets it

        # Subscribers
        self.joint_command_sub = rospy.Subscriber('/robot_joint_command', JointState, self._joint_command_callback, queue_size=1)

        self.rate = rospy.Rate(50) # Publishing rate for /joint_states

        rospy.loginfo("RobotControllerNode initialized. Reading initial joint states...")
        self.initial_setup()

    def initial_setup(self):
        # 1. Read initial positions from hardware
        initial_q_rad = self.dxl_controller.read_positions()
        initial_q_rad_conv = [dxl_units_to_rad(pos, i) for i, pos in enumerate(initial_q_rad)]
        
        if len(initial_q_rad_conv) < 3:
            rospy.logerr(f"Expected 3 initial joint states but got {len(initial_q_rad_conv)}. Check Dynamixel IDs and setup.")
            rospy.signal_shutdown("Initial setup failed.")
            return

        # 2. Update global prismatic value based on initial read
        global _current_q3_prismatic_val
        _current_q3_prismatic_val = initial_q_rad_conv[2]
        rospy.loginfo(f"Initial joint positions (rad/m): {initial_q_rad_conv}")

        # 3. Calculate initial end-effector XY position
        initial_ee_xy = self.robot_kinematics.forward_kinematics_2d(initial_q_rad_conv[0], initial_q_rad_conv[1])
        rospy.loginfo(f"Calculated initial end-effector XY position: {initial_ee_xy}")

        # 4. Publish initial end-effector pose for DXF reader
        ee_pose_msg = Point()
        ee_pose_msg.x = initial_ee_xy[0]
        ee_pose_msg.y = initial_ee_xy[1]
        ee_pose_msg.z = 0.0 # Z is not used for this offset
        self.initial_ee_pose_pub.publish(ee_pose_msg)
        rospy.loginfo("Published initial end-effector pose.")

        # 5. Publish current joint states for visualization
        self._publish_current_joint_states()
        rospy.loginfo("Robot controller ready and waiting for commands.")


    def _joint_command_callback(self, msg: JointState):
        q_command_rad = msg.position
        is_lift_move = (msg.velocity[0] > 0.5) # Using velocity[0] as a flag for lift/drop

        rospy.loginfo(f"Received command: q1={math.degrees(q_command_rad[0]):.2f}deg, q2={math.degrees(q_command_rad[1]):.2f}deg, q3={q_command_rad[2]:.4f}m (Lift/Drop flag: {is_lift_move})")

        # Convert received radians/meters to Dynamixel units
        target_dxl_units = [rad_to_dxl_units(q_command_rad[i], i) for i in range(len(q_command_rad))]

        # Send command to Dynamixel hardware
        self.dxl_controller.write_positions(target_dxl_units)
        
        # Wait for motors to reach the target position
        self.dxl_controller.wait_until_reached(target_dxl_units)
        rospy.loginfo("Command reached by motors.")

        # After movement, publish current (actual) joint states for visualization
        self._publish_current_joint_states()

    def _publish_current_joint_states(self):
        """
        Reads actual positions from Dynamixels and publishes them to /joint_states.
        """
        current_dxl_positions = self.dxl_controller.read_positions()
        current_q_rad_conv = [dxl_units_to_rad(pos, i) for i, pos in enumerate(current_dxl_positions)]
        
        global _current_q3_prismatic_val # Access global variable for prismatic state
        _current_q3_prismatic_val = current_q_rad_conv[2] # Keep it updated

        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = current_q_rad_conv
        joint_state_msg.velocity = []
        joint_state_msg.effort = []
        self.joint_state_pub.publish(joint_state_msg)
        # rospy.loginfo(f"Published actual joint states: {current_q_rad_conv}") # Too chatty for frequent publish

    def run(self):
        while not rospy.is_shutdown():
            # Continuously publish current joint states for visualization,
            # even if no new commands are received.
            self._publish_current_joint_states()
            self.rate.sleep()

        self.dxl_controller.disable_torque_and_close()

if __name__ == '__main__':
    try:
        node = RobotControllerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass