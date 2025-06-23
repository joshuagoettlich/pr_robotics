#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

# --- NEW: Imports needed for Forward Kinematics ---
import numpy as np
import pinocchio as pin
import rospkg
import os

def move_robot_with_fk():
    """
    Calculates the FK for a target pose, prints it, then commands the robot.
    """
    # 1. Initialize the ROS Node
    rospy.init_node('fk_and_publish_node', anonymous=True)

    # --- FORWARD KINEMATICS CALCULATION ---
    rospy.loginfo("--- Calculating Forward Kinematics ---")

    # Define the target joint angles we want to find the position for
    target_angles = np.array([1, -1.0, 0.2])

    try:
        # Find and load the robot's URDF model robustly
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('pr_robotics')
        urdf_path = os.path.join(package_path, 'scripts/urdf/robot.urdf')
        
        rospy.loginfo(f"Loading URDF for FK from: {urdf_path}")
        model = pin.buildModelFromUrdf(urdf_path)
        data = model.createData()
        
        # Perform the Forward Kinematics calculation
        pin.forwardKinematics(model, data, target_angles)
        pin.updateFramePlacements(model, data)
        
        # Get the position of the end-effector frame
        # Assumes the end-effector is the last frame defined in the URDF
        end_effector_frame_id = model.nframes - 1
        end_effector_position = data.oMf[end_effector_frame_id].translation
        
        # Print the result to the console
        rospy.loginfo(f"For Target Joint Angles (rad): {np.round(target_angles, 3)}")
        rospy.loginfo(f"Calculated End-Effector Position (X,Y,Z): {np.round(end_effector_position, 3)}")

    except Exception as e:
        rospy.logerr(f"Failed to perform Forward Kinematics: {e}")
        # Exit if we can't calculate FK, as something is wrong with the model.
        return
    
    rospy.loginfo("--- End of Forward Kinematics Calculation ---")


    # --- ROBOT MOVEMENT COMMAND ---

    # Define the topic and create the publisher
    topic_name = '/three_link_arm_controller/command'
    pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=10)

    # Wait for the controller to be ready
    rospy.loginfo("Waiting for controller subscriber to command movement...")
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.1)

    # Create the Trajectory Message
    traj_msg = JointTrajectory()
    traj_msg.header = Header(stamp=rospy.Time.now())
    traj_msg.joint_names = ['joint1', 'joint2', 'joint3']

    point = JointTrajectoryPoint()
    point.positions = target_angles # Use the same angles we calculated FK for
    point.time_from_start = rospy.Duration(3.0) # 3 seconds to move
    traj_msg.points.append(point)

    # Publish the message
    rospy.loginfo(f"Publishing trajectory goal to move the robot...")
    pub.publish(traj_msg)
    
    rospy.sleep(1)
    rospy.loginfo("Goal published. Script finished.")

if __name__ == '__main__':
    try:
        move_robot_with_fk()
    except rospy.ROSInterruptException:
        pass