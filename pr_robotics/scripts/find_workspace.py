#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import sys
import os
import rospkg
import pinocchio as pin
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

# =============================================================================
# Main Execution Block
# =============================================================================
if __name__ == "__main__":
    try:
        rospy.init_node('workspace_explorer_3d_node', anonymous=True)

        # --- Define 3D Joint Angle Configurations to Test (in radians) ---
        # Each configuration is [joint1, joint2, joint3]
        JOINT_CONFIGS_TO_TEST = [
            np.array([0.0, 0.0, 0.0]),      # Home position
            np.array([0.0, -0.5, 0.1]),     # Arm forward, slightly down
            np.array([0.7, -0.7, 0.2]),     # Arm to the side, bent, Z up
            np.array([-0.7, -0.7, 0.05]),   # Arm to other side, low
            np.array([0.0, 0.0, 0.3]),      # Z-axis fully extended straight up
            np.array([1.0, -1.0, 0.02]),    # The pose you found manually
        ]
        
        # --- Configuration for the full 3D robot ---
        CONTROLLER_TOPIC = '/three_link_arm_controller/command'
        ALL_JOINT_NAMES = ["joint1", "joint2", "joint3"]
        
        # --- Initialization ---
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('pr_robotics')
        # Load the full 3D URDF
        URDF_PATH = os.path.join(package_path, 'scripts/urdf/robot.urdf')
        rospy.loginfo(f"Loading URDF from: {URDF_PATH}")

        # We use Pinocchio for Forward Kinematics
        model = pin.buildModelFromUrdf(URDF_PATH)
        data = model.createData()
        # Track the correct end-effector for the 3D model
        end_effector_frame_id = model.getFrameId("end_effector_link")

        # Setup the simple publisher
        pub = rospy.Publisher(CONTROLLER_TOPIC, JointTrajectory, queue_size=10)
        rospy.loginfo(f"Waiting for controller subscriber on {CONTROLLER_TOPIC}...")
        while pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("Controller subscriber is ready.")

        # --- Main Loop ---
        for i, joint_angles in enumerate(JOINT_CONFIGS_TO_TEST):
            if rospy.is_shutdown(): break
                
            # 1. Calculate the FK for this joint configuration
            pin.forwardKinematics(model, data, joint_angles)
            pin.updateFramePlacements(model, data)
            reachable_pos_3d = data.oMf[end_effector_frame_id].translation

            print("\n" + "="*40)
            print(f"      TESTING CONFIGURATION {i+1}")
            print("="*40)
            print(f"Commanding Joint Angles: {np.round(joint_angles, 3)}")
            print(f"--> CALCULATED REACHABLE XYZ: {np.round(reachable_pos_3d, 3)}")
            
            # 2. Command the robot to move to these angles
            traj_msg = JointTrajectory(header=Header(stamp=rospy.Time.now()), joint_names=ALL_JOINT_NAMES)
            point = JointTrajectoryPoint(positions=joint_angles, time_from_start=rospy.Duration(3.0))
            traj_msg.points.append(point)
            pub.publish(traj_msg)
            
            # Wait for the move to complete
            rospy.sleep(3.5)

        print("\nFinished exploration.")
        print("You can now use the printed 'REACHABLE XYZ' coordinates as targets in the IK script.")

    except (rospy.ROSInterruptException, IOError) as e:
        print(f"Node exiting due to error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")