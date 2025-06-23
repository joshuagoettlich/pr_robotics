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

# === Utility to read single keypress ===
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# === Dynamixel Controller ===
class DynamixelController:
    def __init__(self, ids, position_limits=None, device_name='/dev/ttyUSB0', baudrate=1_000_000, protocol_version=2.0):
        assert isinstance(ids, list), "Motor IDs must be a list."
        self.dxl_ids = ids
        self.device_name = device_name
        self.baudrate = baudrate
        self.protocol_version = protocol_version

        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        self.MOVING_THRESHOLD = 20

        if position_limits is None:
            self.position_limits = [(1100, 2900)] * len(ids)
        else:
            assert len(position_limits) == len(ids), "position_limits must match the number of IDs"
            self.position_limits = position_limits

        self.position_indices = [0] * len(ids)

        self.portHandler = PortHandler(self.device_name)
        self.packetHandler = PacketHandler(self.protocol_version)

        self._open_port()
        self._enable_torque()

    def _open_port(self):
        if not self.portHandler.openPort():
            print("Failed to open port")
            sys.exit(1)
        if not self.portHandler.setBaudRate(self.baudrate):
            print("Failed to set baudrate")
            sys.exit(1)
        print("Port opened and baudrate set")

    def _enable_torque(self):
        for dxl_id in self.dxl_ids:
            result, error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 108, 10)
            self._check_comm(result, error, f"Setting profile acceleration for ID {dxl_id}")
            result, error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 112, 100)
            self._check_comm(result, error, f"Setting profile velocity for ID {dxl_id}")
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            self._check_comm(result, error, f"Enabling torque for ID {dxl_id}")

    def _check_comm(self, result, error, context=""):
        if result != COMM_SUCCESS:
            print(f"{context}: {self.packetHandler.getTxRxResult(result)}")
        elif error != 0:
            print(f"{context}: {self.packetHandler.getRxPacketError(error)}")

    def write_positions(self, target_positions):
        assert len(target_positions) == len(self.dxl_ids), "Length of target_positions must match motor count."
        for i, dxl_id in enumerate(self.dxl_ids):
            goal_pos = target_positions[i]
            result, error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.ADDR_GOAL_POSITION, goal_pos)
            self._check_comm(result, error, f"Writing goal position {goal_pos} for ID {dxl_id}")

    def read_positions(self):
        positions = []
        for dxl_id in self.dxl_ids:
            present_pos, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_POSITION)
            self._check_comm(result, error, f"Reading position for ID {dxl_id}")
            positions.append(present_pos)
        return positions

    def disable_torque_and_close(self):
        for dxl_id in self.dxl_ids:
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            self._check_comm(result, error, f"Disabling torque for ID {dxl_id}")
        self.portHandler.closePort()
        print("Port closed")


# === Pinocchio Robot Wrapper for FK/IK ===
class RobotPinocchio:
    def __init__(self, urdf_path, base_link, end_link, joint_names):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.joint_names = joint_names
        self.base_link = base_link
        self.end_link = end_link

        # Map joint names to indices in model
        self.joint_ids = [self.model.getJointId(name) for name in self.joint_names]

    def forward_kinematics(self, q):
        # q: array of joint angles in radians (only for joints in joint_names)
        # We assume q length == len(joint_names)
        full_q = pin.utils.zero(self.model.nq)
        for i, joint_id in enumerate(self.joint_ids):
            full_q[self.model.joints[joint_id].idx_q()] = q[i]
        pin.forwardKinematics(self.model, self.data, full_q)
        pin.updateFramePlacements(self.model, self.data)
        # Return position of end effector frame
        return self.data.oMf[self.model.getFrameId(self.end_link)].translation

    def inverse_kinematics(self, target_pos, q_seed=None, max_iter=100, tol=1e-4):
        # Simple IK solver for position only using pinocchio's methods
        if q_seed is None:
            q_seed = np.zeros(len(self.joint_names))

        q = q_seed.copy()
        for _ in range(max_iter):
            current_pos = self.forward_kinematics(q)
            error = target_pos - current_pos
            if np.linalg.norm(error) < tol:
                return q
            J = pin.computeFrameJacobian(self.model, self.data, q, self.model.getFrameId(self.end_link))
            J_pos = J[:3, :]  # position part of Jacobian
            try:
                dq = np.linalg.lstsq(J_pos, error, rcond=None)[0]
            except np.linalg.LinAlgError:
                return None
            q += dq
        return None

# === Conversion functions between radians and dynamixel units ===
# Note: Adjust scale and offsets to your robot's specifications

def rad_to_dxl_units(rad, joint_index):
    # Example conversion: assume joint range is -pi to pi maps to dynamixel limits
    # You must replace these limits with your motor's real min/max rad and dxl units
    rad_min = -np.pi
    rad_max = np.pi
    dxl_min, dxl_max = position_limits[joint_index]

    scale = (dxl_max - dxl_min) / (rad_max - rad_min)
    dxl_val = int((rad - rad_min) * scale + dxl_min)
    dxl_val = np.clip(dxl_val, dxl_min, dxl_max)
    return dxl_val

def dxl_units_to_rad(dxl_val, joint_index):
    rad_min = -np.pi
    rad_max = np.pi
    dxl_min, dxl_max = position_limits[joint_index]
    scale = (rad_max - rad_min) / (dxl_max - dxl_min)
    rad = (dxl_val - dxl_min) * scale + rad_min
    return rad

# === ROS Interface for Dynamixel ===
class DynamixelRosInterface:
    def __init__(self, dxl_ids, position_limits):
        self.dxl_controller = DynamixelController(dxl_ids, position_limits)
        self.dxl_ids = dxl_ids
        self.position_limits = position_limits
        self.joint_names = [f"joint{i+1}" for i in range(len(dxl_ids))]

        rospy.init_node('dynamixel_controller_node', anonymous=True)
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.joint_names

    def publish_joint_positions(self, positions):
        # positions: array of dynamixel units (ints)
        self.dxl_controller.write_positions(positions)

        # Convert positions to radians for ROS message
        radians = []
        for i, pos in enumerate(positions):
            rad = dxl_units_to_rad(pos, i)
            radians.append(rad)
        self.joint_state_msg.position = radians
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.joint_state_msg)

    def get_current_joint_positions(self):
        dxl_positions = self.dxl_controller.read_positions()
        # Convert to radians
        radians = []
        for i, pos in enumerate(dxl_positions):
            rad = dxl_units_to_rad(pos, i)
            radians.append(rad)
        return np.array(radians)

    def shutdown(self):
        self.dxl_controller.disable_torque_and_close()

# === Main ===
if __name__ == "__main__":
    try:
        # Define motor IDs and position limits for each motor
        dxl_ids = [1, 2, 3]
        position_limits = [(1200, 2800), (800, 1600), (2800, 3100)]  # example limits for each motor

        # Third joint index controls pen lift z-up
        z_joint_index = 2
        z_min, z_max = position_limits[z_joint_index]
        z_up_angle = z_max
        z_down_angle = z_min

        # Only first two joints for FK/IK
        joint_names = ['joint1', 'joint2']

        # Adjust URDF path, base and end links for your robot
        urdf_path = "/path/to/your_robot.urdf"  # <<<< SET YOUR URDF PATH HERE
        base_link = "base_link"
        end_link = "end_effector_link"

        robot = RobotPinocchio(urdf_path, base_link, end_link, joint_names)
        ros_interface = DynamixelRosInterface(dxl_ids, position_limits)

        rospy.sleep(1.0)  # Wait for ROS setup

        # Example trajectory: list of 3D points (only x,y matter for 2 joints FK/IK)
        aligned_trajectories = [
            np.array([[0.1, 0.1, 0], [0.2, 0.1, 0], [0.2, 0.2, 0]]),  # trajectory 1
            np.array([[0.1, 0.2, 0], [0.15, 0.25, 0], [0.2, 0.3, 0]])  # trajectory 2
        ]

        rate = rospy.Rate(30)

        q_current = ros_interface.get_current_joint_positions()

        print("Starting trajectory following. Press Ctrl+C to stop.")
        for traj in aligned_trajectories:
            for i, point in enumerate(traj):
                if rospy.is_shutdown():
                    break

                if i > 0:
                    dist = np.linalg.norm(point - traj[i - 1])
                    if dist > 0.1:
                        # Lift pen
                        q_move = q_current.copy()
                        q_move[z_joint_index] = z_up_angle
                        ros_interface.publish_joint_positions(q_move)
                        rospy.sleep(0.1)

                # Run IK only for first two joints (x,y)
                q_seed = q_current[:2]
                q_ik_2d = robot.inverse_kinematics(point[:2], q_seed=q_seed)
                if q_ik_2d is not None:
                    # Convert IK radians to dynamixel units for first two joints
                    q_dxl = np.zeros(len(dxl_ids), dtype=int)
                    for j in range(2):
                        q_dxl[j] = rad_to_dxl_units(q_ik_2d[j], j)

                    # Set third joint to pen down
                    q_dxl[z_joint_index] = z_down_angle

                    # Update current joint state in radians for next IK seed
                    q_current = np.array([dxl_units_to_rad(pos, i) for i, pos in enumerate(q_dxl)])

                    ros_interface.publish_joint_positions(q_dxl)
                else:
                    rospy.logwarn(f"IK failed for point {point}")

                rate.sleep()

        print("Trajectory following finished.")

    except rospy.ROSInterruptException:
        pass
    finally:
        ros_interface.shutdown()
