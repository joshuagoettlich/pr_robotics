import os
import numpy as np
import rospkg
import pinocchio as pin

# Initialize rospkg to find package paths
rospack = rospkg.RosPack()

# --- Configuration ---
# Get the path to your 'pr_robotics' package
try:
    package_path = rospack.get_path('pr_robotics')
except rospkg.ResourceNotFound:
    print("Error: 'pr_robotics' package not found. Please ensure it's in your ROS workspace and sourced.")
    print("Attempting to use current working directory as a fallback, but this might not work correctly.")
    package_path = os.getcwd() # Fallback, though likely incorrect for URDF loading

URDF_PATH = os.path.join(package_path, 'scripts/urdf/robot.urdf')

# Check if URDF file exists
if not os.path.exists(URDF_PATH):
    print(f"Error: URDF file not found at {URDF_PATH}. Please verify the path.")
    sys.exit(1)

# Load the full 3D URDF model using Pinocchio
print(f"Loading URDF from: {URDF_PATH}")
try:
    model = pin.buildModelFromUrdf(URDF_PATH)
except Exception as e:
    print(f"Failed to load URDF model: {e}")
    sys.exit(1)

data = model.createData() # Create data structure for computations
NQ = model.nq # Number of generalized coordinates
NV = model.nv # Number of generalized velocities (tangent space dimension)

print('Joints order in the model tree: ')
for name in model.names:
    print(name)
print()

# Get the ID of the end-effector frame
id_ee = model.getFrameId('end_effector_link')
if id_ee == model.nframes: # pin.nframes is returned if not found
    print("Error: 'end_effector_link' frame not found in the URDF. Please check the link name.")
    sys.exit(1)
print(f'End-effector frame ID: {id_ee}')

# --- Inverse Kinematics Solver Function ---

def solve_ik(model, initial_q, target_position, target_rpy=None, max_iterations=10000, tolerance=1e-4, dt=0.1, position_only=False):
    """
    Solves Inverse Kinematics for the end-effector to reach a target pose (position and/or orientation).

    Args:
        model (pin.Model): The Pinocchio model of the robot.
        initial_q (np.ndarray): The initial guess for the joint configuration.
        target_position (np.ndarray): Desired (x, y, z) position of the end-effector.
        target_rpy (np.ndarray, optional): Desired (roll, pitch, yaw) orientation of the end-effector in radians.
                                          Required if position_only is False. Defaults to None.
        max_iterations (int): Maximum number of iterations for the solver.
        tolerance (float): Convergence tolerance for the error.
        dt (float): Step size for updating joint positions.
        position_only (bool): If True, only solves for position, ignoring target_rpy.

    Returns:
        np.ndarray or None: The calculated joint configuration (q) if converged, else None.
    """
    data = model.createData() # Create data object for this IK run
    q = initial_q.copy() # Start with the initial guess

    # Check if target_rpy is provided when position_only is False
    if not position_only and target_rpy is None:
        raise ValueError("target_rpy must be provided if position_only is False.")

    if not position_only:
        # Convert target RPY to a rotation matrix only if orientation is relevant
        target_rotation = pin.rpy.rpyToMatrix(target_rpy[0], target_rpy[1], target_rpy[2])

    print(f"\n--- Starting IK Solver ---")
    print(f"Target Position: {target_position}")
    print(f"Initial q: {q}")
    if position_only:
        print("Solving for position only (orientation is not constrained).")
    else:
        print(f"Target Orientation (RPY): {target_rpy}")
        print("Solving for position and orientation.")


    for i in range(max_iterations):
        # 1. Compute forward kinematics and Jacobian for current q
        # pin.computeAllTerms is generally good practice to ensure all data is up-to-date
        pin.computeAllTerms(model, data, q, np.zeros(model.nv))
        pin.updateFramePlacements(model, data) # Update all frame placements

        # 2. Get current end-effector pose
        current_pose = data.oMf[id_ee]
        current_pos = current_pose.translation
        # current_rot = current_pose.rotation # Only needed for orientation error

        # 3. Calculate error and select Jacobian part based on 'position_only' flag
        pos_error = target_position - current_pos # Always calculate position error

        J_full = pin.getFrameJacobian(model, data, id_ee, pin.LOCAL_WORLD_ALIGNED)

        if position_only:
            error = pos_error # Error is just the 3D position error
            J = J_full[:3, :] # Use only the linear part of the Jacobian (first 3 rows)
        else:
            # Using pin.log6 on the relative transform from current to target
            # This is more robust for Lie groups
            error_transform = current_pose.inverse() * pin.SE3(target_rotation, target_position)
            error_twist = pin.log6(error_transform) # This gives a 6D twist (linear and angular error)

            linear_error = error_twist.linear
            angular_error = error_twist.angular

            # Combine linear and angular errors into a 6D error vector
            error = np.concatenate((linear_error, angular_error))
            J = J_full # Use the full 6xN Jacobian

        # 4. Use pseudo-inverse for Jacobian
        J_pinv = np.linalg.pinv(J)

        # 5. Calculate joint velocity update (dq)
        dq = J_pinv @ error

        # 6. Update joint positions (q)
        q += dt * dq

        # 7. Clamp joint angles to model limits (simple joint limit handling)
        q = np.maximum(model.lowerPositionLimit, np.minimum(model.upperPositionLimit, q))

        # 8. Check for convergence
        current_error_norm = np.linalg.norm(error)
        # print(f"  Iteration {i+1}: Error = {current_error_norm:.6f}") # Uncomment for per-iteration error
        if current_error_norm < tolerance:
            print(f"IK converged after {i+1} iterations. Final Error: {current_error_norm:.6f}")
            return q

    print(f"IK did NOT converge after {max_iterations} iterations. Final Error: {current_error_norm:.6f}")
    return None

# --- Example Usage ---

# 1. Define an initial guess for the joint configuration
# (q1, q2, q3) - make sure this matches your URDF joints
# Example: q1=0, q2=0, q3=0 (all joints at home position)
initial_q = np.array([0.0, 0.0, 0.0]) # Adjust based on your robot's actual joint order and limits

# 2. Define the desired end-effector pose
# Target Position (x, y, z)
desired_position = np.array([0.3, 0.2, 0.2]) # Example: 0.3m x, 0.2m y, 0.5m z

# Target Orientation (Roll, Pitch, Yaw in radians) - This will be ignored when position_only=True
# It's kept here just as a reference if you ever need to switch back to full pose IK.
desired_rpy = np.array([0.0, 0.0, np.pi/2]) # Example: 90 degrees rotation around Z-axis

# Running IK for Position Only
print("\n=== Running IK for Position Only ===")
# Note: desired_rpy is provided but will be ignored because position_only=True
solution_q_pos_only = solve_ik(model, initial_q, desired_position, position_only=True)

if solution_q_pos_only is not None:
    print(f"\nCalculated Joint Configuration (q) for Position Only:")
    print(solution_q_pos_only)

    # Verify the achieved pose with the calculated q
    print("\n--- Verifying Achieved Position ---")
    pin.computeAllTerms(model, data, solution_q_pos_only, np.zeros(model.nv))
    pin.updateFramePlacements(model, data)

    achieved_pos_ee_pos_only = data.oMf[id_ee].translation
    achieved_R_ee_pos_only = data.oMf[id_ee].rotation # Still get rotation to show it's not controlled
    achieved_euler_angles_ee_pos_only = pin.rpy.matrixToRpy(achieved_R_ee_pos_only)

    print(f'Achieved End-effector Position: {achieved_pos_ee_pos_only}')
    print(f'Achieved End-effector Orientation (Euler angles): {achieved_euler_angles_ee_pos_only}')
    # For position-only, only check linear error against the desired position
    final_pos_error_norm = np.linalg.norm(desired_position - achieved_pos_ee_pos_only)
    print(f"Final Position Error Norm (Verification): {final_pos_error_norm:.6f}")

else:
    print("\nIK solver for Position Only failed to find a solution.")
