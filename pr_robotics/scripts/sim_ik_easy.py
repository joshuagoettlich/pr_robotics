#!/usr/bin/env python3

import rospy
import math
from typing import Tuple, Optional
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# --- Your existing IK function (copy-pasted for clarity) ---
def ik_2dof_with_limits(L1: float, L2: float, x: float, y: float) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """
    Solves the inverse kinematics for a 2DOF planar arm with angle limits.
    Returns two possible (theta1, theta2) solutions in degrees within [0, 170], or None if no valid solution exists.

    Parameters:
    - L1, L2: link lengths
    - x, y: target position

    Returns:
    - Tuple of two valid configurations ((theta1_down, theta2_down), (theta1_up, theta2_up))
    """
    r_squared = x**2 + y**2
    r = math.sqrt(r_squared)

    # Check reachability
    if r > L1 + L2 or r < abs(L1 - L2):
        rospy.logwarn(f"Target ({x}, {y}) is unreachable. r: {r}, L1+L2: {L1+L2}, |L1-L2|: {abs(L1-L2)}")
        return None

    # Compute elbow angle using the Law of Cosines
    cos_theta2 = (r_squared - L1**2 - L2**2) / (2 * L1 * L2)

    if abs(cos_theta2) > 1.0:
        rospy.logwarn(f"No real solution for theta2. cos_theta2: {cos_theta2}")
        return None  # No real solution

    theta2_rad = math.acos(cos_theta2)
    theta2_deg_down = math.degrees(theta2_rad)
    theta2_deg_up = math.degrees(-theta2_rad)

    # Compute shoulder angle
    phi = math.atan2(y, x)
    # Ensure denominator is not zero
    if r == 0:
        psi = 0
    else:
        arg_psi = (r_squared + L1**2 - L2**2) / (2 * L1 * r)
        if abs(arg_psi) > 1.0:
            rospy.logwarn(f"No real solution for psi. arg_psi: {arg_psi}")
            return None # Should ideally not happen if r checks are good, but for safety
        psi = math.acos(arg_psi)

    theta1_rad_down = phi - psi
    theta1_rad_up = phi + psi

    theta1_deg_down = math.degrees(theta1_rad_down) % 360
    theta1_deg_up = math.degrees(theta1_rad_up) % 360
    theta2_deg_down %= 360
    theta2_deg_up %= 360

    # Apply limits (using the 0 to 170 degrees limits as per your original function)
    # Note: These limits are stricter than the URDF's -1.5 to 1.5 radians (~-86 to 86 degrees)
    # You might want to harmonize these or choose one set.
    def in_limits(theta1_deg, theta2_deg):
        return 0 <= theta1_deg <= 170 and 0 <= theta2_deg <= 170

    result = []
    solution_down = None
    if in_limits(theta1_deg_down, theta2_deg_down):
        solution_down = (theta1_deg_down, theta2_deg_down)
    else:
        rospy.logdebug(f"Solution 'down' ({theta1_deg_down:.2f}, {theta2_deg_down:.2f}) out of [0, 170] limits.")

    solution_up = None
    if in_limits(theta1_deg_up, theta2_deg_up):
        solution_up = (theta1_deg_up, theta2_deg_up)
    else:
        rospy.logdebug(f"Solution 'up' ({theta1_deg_up:.2f}, {theta2_deg_up:.2f}) out of [0, 170] limits.")

    if solution_down is None and solution_up is None:
        return None
    return (solution_down, solution_up)
# --- End of your existing IK function ---


class IKRobotController:
    def __init__(self):
        rospy.init_node('ik_robot_controller', anonymous=True)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(10) # 10 Hz

        # Link lengths from your robot's design (adjust if your URDF dimensions differ)
        # These are crucial and should accurately reflect your robot's geometry.
        # Based on your URDF:
        # L1: distance from joint1 to joint2 along the arm.
        #     joint2 origin xyz="-0.0341 0.230 0.0" relative to link1.
        #     The significant length is in the Y direction (0.230) after the rotation.
        #     Assuming your link1.STL and link2.STL are designed to align this.
        #     If link1 is 0.23m long and link2 is 0.17m long (to end of link2 for 2DOF part before prismatic)
        #     You'll need to measure these accurately from your CAD or URDF.
        #     For simplicity, let's assume L1 = 0.230 and L2 = 0.170 as implied by origins.
        self.L1 = 0.230 # Approximate length of link1 from joint1 to joint2
        self.L2 = 0.2056 # Approximate length of link2 from joint2 to end of link2's arm part

        rospy.loginfo(f"Robot IK Controller initialized with L1={self.L1}, L2={self.L2}")

    def set_joint_positions(self, q1_rad, q2_rad, q3_prismatic=0.0):
        """
        Publishes joint commands to the /joint_states topic.
        Note: For actual control in Gazebo, you usually publish to topics like
        /three_link_robot/joint1_position_controller/command
        if you have ros_control position controllers set up.
        Publishing to /joint_states directly like this often only works for visualizers
        or very basic simulated robots without proper controllers.
        """
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = ['joint1', 'joint2', 'joint3'] # Match URDF joint names
        joint_state_msg.position = [q1_rad, q2_rad, q3_prismatic]
        joint_state_msg.velocity = []
        joint_state_msg.effort = []

        self.joint_pub.publish(joint_state_msg)
        rospy.loginfo(f"Published joint positions: joint1={math.degrees(q1_rad):.2f}deg, joint2={math.degrees(q2_rad):.2f}deg, joint3={q3_prismatic:.2f}m")

    def run(self):
        # Example target (x, y) coordinates for the end-effector
        # Try to reach a point. Adjust these based on your robot's workspace.
        target_x = 0.2
        target_y = 0.3

        # Convert target to robot base frame if necessary (assuming x,y are in robot's base frame)
        # Note: Your URDF has 'base_link' as the origin, so this (x,y) is relative to it.
        # However, joint1 has an origin `xyz="0.0011 0.07051 0.11867"` relative to base_link.
        # This means your IK's (x,y) should be relative to `joint1`'s origin.
        # For a truly planar 2DOF, the base should be at (0,0) or your (x,y) should be compensated.
        # Let's assume for now your IK's (x,y) is relative to the actual shoulder joint.

        # Adjust target based on the offset of joint1 from base_link
        # joint1_origin_x = 0.0011
        # joint1_origin_y = 0.07051
        # compensated_x = target_x - joint1_origin_x
        # compensated_y = target_y - joint1_origin_y
        # For simplicity in this example, let's proceed assuming the IK is relative to base, and base offset is small in xy plane, or accounted for.
        # For accurate control, (x,y) input to IK should be relative to the joint1's pivot.
        # If your target_x, target_y are in the base_link frame, then:
        #  x_ik = target_x - 0.0011
        #  y_ik = target_y - 0.07051

        # Let's use simple target_x, target_y for now, assuming the IK is valid for these.
        rospy.loginfo(f"Attempting to reach target: (x={target_x:.2f}, y={target_y:.2f})")

        solutions = ik_2dof_with_limits(self.L1, self.L2, target_x, target_y)

        if solutions:
            rospy.loginfo("IK Solutions found:")
            valid_solutions = [sol for sol in solutions if sol is not None]

            if not valid_solutions:
                rospy.logwarn("No valid IK solutions within the specified angle limits [0, 170] degrees.")
                return

            # Choose one solution (e.g., the first valid one)
            chosen_solution = valid_solutions[0]
            theta1_deg, theta2_deg = chosen_solution

            rospy.loginfo(f"Chosen solution (degrees): Theta1={theta1_deg:.2f}, Theta2={theta2_deg:.2f}")

            # Convert to radians
            theta1_rad = math.radians(theta1_deg)
            theta2_rad = math.radians(theta2_deg)

            # Check against URDF limits if desired
            # URDF limits for joint1 and joint2 are [-1.5, 1.5] radians (~ -86 to 86 degrees)
            # Your IK limits are [0, 170] degrees (~ 0 to 2.967 radians)
            # The IK limits are stricter in lower bound (0 vs -86) and wider in upper bound (170 vs 86)
            # You must ensure the IK solution (in degrees [0,170]) translates to valid URDF radians.
            # Example: A 100-degree solution from IK is valid for IK but not for URDF's 86-degree limit.
            urdf_lower_limit_rad = -1.5
            urdf_upper_limit_rad = 1.5

            if not (urdf_lower_limit_rad <= theta1_rad <= urdf_upper_limit_rad and \
                    urdf_lower_limit_rad <= theta2_rad <= urdf_upper_limit_rad):
                rospy.logwarn(f"Calculated IK solution ({math.degrees(theta1_rad):.2f}deg, {math.degrees(theta2_rad):.2f}deg) is OUTSIDE URDF joint limits [-1.5, 1.5] rad!")
                rospy.logwarn("This means the robot in simulation might not reach the target or will be clamped.")
                # You might choose to clamp the values or skip publishing
                # For this example, we will proceed but be aware of this.

            # Publish joint positions
            # Assuming joint3 (prismatic) is at its home position (e.g., 0.0)
            self.set_joint_positions(theta1_rad, theta2_rad, q3_prismatic=0.0)

        else:
            rospy.logerr(f"No valid IK solution found for target ({target_x}, {target_y}).")

        rospy.spin() # Keep the node running

if __name__ == '__main__':
    try:
        controller = IKRobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass