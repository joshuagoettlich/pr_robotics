#!/usr/bin/env python3

import rospy
import math
import numpy as np
from typing import Tuple, Optional
from sensor_msgs.msg import JointState # For publishing joint commands
from pr_robotics_msgs.msg import Trajectory2D # For subscribing to trajectories
from geometry_msgs.msg import Point # For points within the trajectory

# --- Your existing IK function (copied and adapted) ---
def ik_2dof_with_limits(L1: float, L2: float, x: float, y: float) -> Optional[Tuple[float, float]]:
    """
    Solves the inverse kinematics for a 2DOF planar arm with angle limits.
    Returns a single (theta1, theta2) solution in radians, or None if no valid solution exists.
    It prefers the 'elbow-down' solution.

    Parameters:
    - L1, L2: link lengths
    - x, y: target position (relative to the shoulder joint)

    Returns:
    - Tuple of (theta1_rad, theta2_rad) if a valid solution is found, otherwise None.
    """
    r_squared = x**2 + y**2
    r = math.sqrt(r_squared)

    # Check reachability
    if r > L1 + L2 or r < abs(L1 - L2):
        rospy.logwarn(f"IK: Target ({x:.3f}, {y:.3f}) is unreachable. r: {r:.3f}, L1+L2: {L1+L2:.3f}, |L1-L2|: {abs(L1-L2):.3f}")
        return None

    # Compute elbow angle using the Law of Cosines
    cos_theta2 = (r_squared - L1**2 - L2**2) / (2 * L1 * L2)

    if abs(cos_theta2) > 1.0:
        rospy.logwarn(f"IK: No real solution for theta2. cos_theta2: {cos_theta2:.3f}")
        return None

    theta2_rad_down = math.acos(cos_theta2)
    theta2_rad_up = -math.acos(cos_theta2) # Elbow up and elbow down

    # Compute shoulder angle
    phi = math.atan2(y, x)
    psi = math.acos((r_squared + L1**2 - L2**2) / (2 * L1 * r)) if r != 0 else 0

    theta1_rad_down = phi - psi
    theta1_rad_up = phi + psi

    # Define URDF limits for revolute joints (approx -86 to 86 degrees)
    urdf_lower_limit_rad = -1.5
    urdf_upper_limit_rad = 1.5

    def is_within_urdf_limits(q1_rad, q2_rad):
        return (urdf_lower_limit_rad <= q1_rad <= urdf_upper_limit_rad and
                urdf_lower_limit_rad <= q2_rad <= urdf_upper_limit_rad)

    # Prioritize elbow-down solution
    if is_within_urdf_limits(theta1_rad_down, theta2_rad_down):
        rospy.logdebug(f"IK: Chose elbow-down: q1={math.degrees(theta1_rad_down):.2f}deg, q2={math.degrees(theta2_rad_down):.2f}deg")
        return (theta1_rad_down, theta2_rad_down)
    elif is_within_urdf_limits(theta1_rad_up, theta2_rad_up):
        rospy.logdebug(f"IK: Chose elbow-up: q1={math.degrees(theta1_rad_up):.2f}deg, q2={math.degrees(theta2_rad_up):.2f}deg")
        return (theta1_rad_up, theta2_rad_up)
    else:
        rospy.logwarn(f"IK: No valid solution found within URDF limits.")
        rospy.logdebug(f"IK: Elbow-down: ({math.degrees(theta1_rad_down):.2f}deg, {math.degrees(theta2_rad_down):.2f}deg)")
        rospy.logdebug(f"IK: Elbow-up: ({math.degrees(theta1_rad_up):.2f}deg, {math.degrees(theta2_rad_up):.2f}deg)")
        return None

# --- End of your existing IK function ---


# === Robot Kinematics and Joint Offset ===
class RobotKinematics:
    """Manages robot specific kinematics, link lengths, and base offsets."""
    def __init__(self):
        # Link lengths from your robot's design. These are crucial.
        self.L1 = 0.230 # Approximate length of link1 from joint1 to joint2
        self.L2 = 0.2056 # Approximate length of link2 from joint2 to end of link2's arm part
        rospy.loginfo(f"Robot Kinematics initialized with L1={self.L1}, L2={self.L2}")

        # Offset of joint1 from the base_link in the XY plane.
        # This needs to be precisely determined from your URDF or CAD.
        # joint1 origin xyz="0.0011 0.07051 0.11867" relative to base_link in your URDF.
        # Assuming your 2D IK is in the XY plane and joint1 is the shoulder pivot.
        self.joint1_offset_xy = np.array([0.0011, 0.07051])
        rospy.loginfo(f"Joint1 offset from base_link (XY): {self.joint1_offset_xy}")

    def get_target_relative_to_joint1(self, target_xy_base_frame: np.ndarray) -> np.ndarray:
        """
        Adjusts a target XY position from the robot's base frame to be relative to joint1's pivot.
        """
        return target_xy_base_frame - self.joint1_offset_xy

class IKSolverNode:
    def __init__(self):
        rospy.init_node('ik_solver_node', anonymous=True)

        self.robot_kinematics = RobotKinematics()

        # Publishers
        self.joint_command_pub = rospy.Publisher('/robot_joint_command', JointState, queue_size=1)
        # Subscribers
        self.trajectory_sub = rospy.Subscriber('/dxf_trajectories', Trajectory2D, self._trajectory_callback, queue_size=1)
        
        # Prismatic Z-axis control variables (passed to robot_controller_node)
        # These are constants set by the IK solver based on task requirements
        self.z_joint_index = 2 # Assuming joint3 is the prismatic joint (0-indexed)
        self.z_up_val_m = rospy.get_param('~z_up_val_m', 0.05) # Default pen up height in meters
        self.z_down_val_m = rospy.get_param('~z_down_val_m', 0.0) # Default pen down height in meters
        
        rospy.loginfo(f"IKSolverNode initialized. Z_up_val_m: {self.z_up_val_m}, Z_down_val_m: {self.z_down_val_m}")
        rospy.loginfo("Waiting for DXF trajectories...")

        self.current_trajectory_points = []
        self.processing_trajectory = False
        self.last_published_prismatic_state = self.z_up_val_m # Start with pen up logically

    def _trajectory_callback(self, msg: Trajectory2D):
        if self.processing_trajectory:
            rospy.logwarn("Received new trajectory while still processing previous one. Skipping.")
            return

        rospy.loginfo(f"Received new trajectory with {len(msg.points)} points.")
        self.current_trajectory_points = msg.points
        self.processing_trajectory = True
        self._process_trajectory()
        self.processing_trajectory = False
        rospy.loginfo("Finished processing trajectory.")

    def _process_trajectory(self):
        # 1. Command pen UP before moving to the first point of a new trajectory
        self._publish_full_joint_command(0.0, 0.0, self.z_up_val_m, is_lift_move=True) # q1, q2 don't matter much for lift, but keep robot stable
        # robot_controller_node will handle waiting for this to be reached

        for i, point_msg in enumerate(self.current_trajectory_points):
            if rospy.is_shutdown():
                break

            target_xy_base_frame = np.array([point_msg.x, point_msg.y])
            rospy.loginfo(f"Processing point {i+1}/{len(self.current_trajectory_points)}: Target XY (base frame): {target_xy_base_frame}")

            # Get target relative to the shoulder joint for IK function
            target_xy_relative_to_joint1 = self.robot_kinematics.get_target_relative_to_joint1(target_xy_base_frame)

            # Run 2DOF IK
            ik_solution_2d_rad = ik_2dof_with_limits(
                self.robot_kinematics.L1, self.robot_kinematics.L2,
                target_xy_relative_to_joint1[0], target_xy_relative_to_joint1[1]
            )

            if ik_solution_2d_rad is not None:
                q1_rad, q2_rad = ik_solution_2d_rad
                
                # For the first point of the trajectory, command pen DOWN
                # For subsequent points, keep pen DOWN
                prismatic_z = self.z_down_val_m if i == 0 else self.z_down_val_m

                self._publish_full_joint_command(q1_rad, q2_rad, prismatic_z, is_lift_move=False)
            else:
                rospy.logwarn(f"IK failed for point ({point_msg.x:.3f}, {point_msg.y:.3f}), skipping.")
            
            # rate.sleep() is removed here; robot_controller_node will handle waiting for completion
            # before it reports back/moves to next point. This topic is fire-and-forget for IK.

        # Command pen UP after finishing the trajectory
        self._publish_full_joint_command(0.0, 0.0, self.z_up_val_m, is_lift_move=True) # Final lift

    def _publish_full_joint_command(self, q1_rad: float, q2_rad: float, q3_prismatic_val_m: float, is_lift_move: bool):
        """
        Publishes the full 3-joint command to robot_controller_node.
        """
        joint_cmd_msg = JointState()
        joint_cmd_msg.header.stamp = rospy.Time.now()
        joint_cmd_msg.name = ['joint1', 'joint2', 'joint3'] # Match names in robot_controller
        joint_cmd_msg.position = [q1_rad, q2_rad, q3_prismatic_val_m]
        
        # Add a flag to indicate if this is a pen lift/drop move,
        # so robot_controller_node can treat it specially (e.g., waiting for completion)
        # We can use an unused field or define a custom message for more complex flags.
        # For simplicity, let's use velocity[0] as a flag: 0 for normal, 1 for lift/drop
        joint_cmd_msg.velocity = [1.0 if is_lift_move else 0.0, 0.0, 0.0]

        rospy.loginfo(f"IK: Publishing command: q1={math.degrees(q1_rad):.2f}deg, q2={math.degrees(q2_rad):.2f}deg, q3={q3_prismatic_val_m:.4f}m (Lift/Drop: {is_lift_move})")
        self.joint_command_pub.publish(joint_cmd_msg)
        
        # In this distributed model, IK solver just sends. The controller waits.

    def run(self):
        rospy.spin() # Keep the node alive

if __name__ == '__main__':
    try:robot_controller_node.py
        node = IKSolverNode()
        node.run()
    except rospy.ROSInterruptException:
        pass