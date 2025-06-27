import math
import numpy as np
import rospy
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


def solve_ik_with_closest_solution(L1, L1_offset, L2, L2_offset, x, y, current_q_rad):
    """
    Solves the inverse kinematics for the 2-DOF arm with dual offsets using the Law of Cosines.
    
    This implementation is a Python translation of the robust MATLAB example provided.
    It calculates both possible elbow solutions and selects the one closest to the
    current joint configuration to ensure smooth trajectories.

    Args:
        L1 (float): Length of link 1.
        L1_offset (float): Perpendicular offset on link 1.
        L2 (float): Primary length of link 2.
        L2_offset (float): Perpendicular offset on link 2.
        x (float): Target x-coordinate in the frame of joint 1.
        y (float): Target y-coordinate in the frame of joint 1.
        current_q_rad (tuple[float, float]): A tuple (q1, q2) of the current joint
                                             angles in radians. Used to select the
                                             closest valid solution.

    Returns:
        tuple[float, float] | None: A tuple of (theta1_rad, theta2_rad) if a
                                     solution is found, otherwise None.
    """
    # --- 0. Pre-calculate effective lengths and offset angles (alpha, beta) ---
    L_eff_1 = math.sqrt(L1**2 + L1_offset**2)
    alpha = math.atan2(L1_offset, L1)  # Offset angle for Link 1

    L_eff_2 = math.sqrt(L2**2 + L2_offset**2)
    beta = math.atan2(L2_offset, L2)   # Offset angle for Link 2

    # --- 1. Check Physical Reachability ---
    D_sq = x**2 + y**2
    max_reach_sq = (L_eff_1 + L_eff_2)**2
    min_reach_sq = (L_eff_1 - L_eff_2)**2

    if D_sq > max_reach_sq or D_sq < min_reach_sq:
        # rospy.logwarn_throttle(5, "IK: Target is physically unreachable.")
        return None

    # --- 2. Solve for Both Elbow Configurations using Law of Cosines ---
    # This section calculates the angles (phi1, phi2) for a simplified, non-offset arm.
    
    # Use max/min to clamp value due to potential floating point inaccuracies
    cos_phi2 = max(-1.0, min(1.0, (D_sq - L_eff_1**2 - L_eff_2**2) / (2 * L_eff_1 * L_eff_2)))
    
    # Solution 1 (e.g., Elbow Down)
    phi2_sol1 = math.acos(cos_phi2)
    phi1_sol1 = math.atan2(y, x) - math.atan2(L_eff_2 * math.sin(phi2_sol1), L_eff_1 + L_eff_2 * math.cos(phi2_sol1))
    
    # Solution 2 (e.g., Elbow Up)
    phi2_sol2 = -math.acos(cos_phi2)
    phi1_sol2 = math.atan2(y, x) - math.atan2(L_eff_2 * math.sin(phi2_sol2), L_eff_1 + L_eff_2 * math.cos(phi2_sol2))

    # --- 3. Convert simplified angles (phi) back to the real robot angles (q) ---
    # This is the critical step that accounts for the L-shapes of the links.
    q1_sol1 = phi1_sol1 - alpha
    q2_sol1 = phi2_sol1 + alpha - beta  # Note: This is the formula from the MATLAB example

    q1_sol2 = phi1_sol2 - alpha
    q2_sol2 = phi2_sol2 + alpha - beta
    
    # --- 4. Normalize angles to [-pi, pi] for consistent checks ---
    solutions = [
        [(q1_sol1 + np.pi) % (2 * np.pi) - np.pi, (q2_sol1 + np.pi) % (2 * np.pi) - np.pi],
        [(q1_sol2 + np.pi) % (2 * np.pi) - np.pi, (q2_sol2 + np.pi) % (2 * np.pi) - np.pi]
    ]

    # --- 5. Check Joint Limits ---
    urdf_limit = np.pi / 2 # Using the limit from your original code
    
    valid_solutions = []
    for q1, q2 in solutions:
        if (-urdf_limit <= q1 <= urdf_limit) and (-urdf_limit <= q2 <= urdf_limit):
            valid_solutions.append((q1, q2))

    # --- 6. Select the Best Valid Solution ---
    if not valid_solutions:
        # rospy.logwarn_throttle(5, "IK: Target in workspace, but unreachable due to joint limits.")
        return None
    
    if len(valid_solutions) == 1:
        return valid_solutions[0]
    
    # If both solutions are valid, pick the one closer to the current joint configuration
    dist1_sq = (valid_solutions[0][0] - current_q_rad[0])**2 + (valid_solutions[0][1] - current_q_rad[1])**2
    dist2_sq = (valid_solutions[1][0] - current_q_rad[0])**2 + (valid_solutions[1][1] - current_q_rad[1])**2

    if dist1_sq <= dist2_sq:
        return valid_solutions[0]
    else:
        return valid_solutions[1]
    

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

self.L2 =0.038 
        self.L2_offset =  0.2056 # The 38mm x-offset for L2
class RobotKinematics:
    def __init__(self):
        self.L1 = 0.230
        self.L1_offset = -0.031 # The -31mm offset for L1
        
        self.joint1_offset_xy =np.array([0.00, 0.0])
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

