#!/usr/bin/env python3

import rospy
import numpy as np
import ezdxf
import sys
import matplotlib.pyplot as plt # For optional plotting

from pr_robotics_msgs.msg import Trajectory2D # Our custom message
from geometry_msgs.msg import Point # For individual points in the trajectory
from std_msgs.msg import Header # For the message header

EZDXF_AVAILABLE = False
try:
    import ezdxf
    EZDXF_AVAILABLE = True
except ImportError:
    rospy.logerr("The 'ezdxf' library is required to read trajectories from DXF files. Please install it using: pip install ezdxf")
MATPLOTLIB_AVAILABLE = False
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    rospy.logwarn("The 'matplotlib' library is recommended for plotting trajectories. Please install it using: pip install matplotlib")

def plot_trajectories(trajectories, offset=[0,0]):
    """
    Plots the extracted trajectories using Matplotlib for a visual preview.
    """
    if not MATPLOTLIB_AVAILABLE:
        rospy.logwarn("Matplotlib not found. Skipping trajectory plot.")
        return

    offset_array = np.array(offset)

    plt.figure(figsize=(10, 8))
    ax = plt.gca()

    for i, traj in enumerate(trajectories):
        offset_traj = traj + offset_array
        ax.plot(offset_traj[:, 0], offset_traj[:, 1], marker='.', label=f'Trajectory {i+1}')
        ax.plot(offset_traj[0, 0], offset_traj[0, 1], 'go', markersize=10, label=f'_Start {i+1}')
        ax.plot(offset_traj[-1, 0], offset_traj[-1, 1], 'ro', markersize=10, label=f'_End {i+1}')

    ax.set_aspect('equal', adjustable='box')
    plt.title('Extracted DXF Trajectories (Preview)')
    plt.xlabel(f'X coordinate (m) [Offset: {offset[0]}]')
    plt.ylabel(f'Y coordinate (m) [Offset: {offset[1]}]')
    plt.grid(True)

    rospy.loginfo("Displaying trajectory plot. Close the plot window to continue.")
    plt.show()


def extract_trajectories_from_dxf_in_meters(filepath):
    """
    Extracts trajectories from a DXF file and normalizes their positions.
    """
    if not EZDXF_AVAILABLE:
        rospy.logerr("ezdxf library not available. Cannot extract trajectories from DXF.")
        return []

    raw_trajectories = []
    try:
        doc = ezdxf.readfile(filepath)
        msp = doc.modelspace()

        for entity in msp:
            points = []
            if entity.dxftype() == 'LWPOLYLINE':
                for point in entity.get_points(format='xy'):
                    point_in_meters = [point[0] / 1000.0, point[1] / 1000.0]
                    points.append(point_in_meters)

            elif entity.dxftype() == 'POLYLINE':
                if not entity.is_3d_polyline and not entity.is_closed:
                    for vertex in entity.vertices:
                        point_in_meters = [vertex.dxf.location.x / 1000.0, vertex.dxf.location.y / 1000.0]
                        points.append(point_in_meters)

            if points:
                raw_trajectories.append(np.array(points))

    except IOError:
        rospy.logerr(f"Error: Could not find or open the DXF file at '{filepath}'")
        return []
    except ezdxf.DXFStructureError:
        rospy.logerr(f"Error: The DXF file at '{filepath}' is invalid or corrupt.")
        return []

    if not raw_trajectories:
        rospy.logwarn(f"No valid trajectories found in {filepath}.")
        return []

    global_offset = raw_trajectories[0][0]
    normalized_trajectories = [traj - global_offset for traj in raw_trajectories]

    rospy.loginfo(f"Successfully extracted and normalized {len(normalized_trajectories)} trajectories from {filepath}.")
    return normalized_trajectories

class DXFReaderNode:
    def __init__(self):
        rospy.init_node('dxf_reader_node', anonymous=True)

        self.trajectory_pub = rospy.Publisher('/dxf_trajectories', Trajectory2D, queue_size=1)
        self.rate = rospy.Rate(1) # Publish once per second, or on demand

        # --- Configuration ---
        self.dxf_path = rospy.get_param('~dxf_path', './Not_so_much_pain_afterall.dxf')
        self.plot_preview = rospy.get_param('~plot_preview', True)
        self.initial_ee_xy_offset = rospy.get_param('~initial_ee_xy_offset', [0.0, 0.0]) # This will be set by robot_controller_node

        rospy.loginfo(f"DXFReaderNode initialized. DXF Path: {self.dxf_path}")
        rospy.loginfo("Waiting for robot_controller to provide initial end-effector offset...")

        # We'll use a OneShotSubscriber to get the initial_ee_xy_offset from robot_controller_node
        # This allows robot_controller_node to start first, determine its position, and then tell this node.
        self.offset_received = False
        rospy.Subscriber('/robot_initial_ee_pose', Point, self._initial_ee_pose_callback, queue_size=1)


    def _initial_ee_pose_callback(self, msg):
        if not self.offset_received:
            self.initial_ee_xy_offset = [msg.x, msg.y]
            rospy.loginfo(f"Received initial end-effector XY offset: {self.initial_ee_xy_offset}")
            self.offset_received = True
            # Once offset is received, immediately process and publish trajectories
            self.process_and_publish_trajectories()

    def process_and_publish_trajectories(self):
        if not self.offset_received:
            rospy.logwarn("Initial end-effector offset not yet received. Cannot process trajectories.")
            return

        all_trajectories = extract_trajectories_from_dxf_in_meters(self.dxf_path)

        if not all_trajectories:
            rospy.logerr("No trajectories extracted. Exiting DXF reader.")
            return

        if self.plot_preview:
            plot_trajectories(all_trajectories, offset=self.initial_ee_xy_offset)

        # Publish each trajectory as a separate message
        rospy.loginfo(f"Publishing {len(all_trajectories)} trajectories...")
        for traj_idx, raw_traj_points in enumerate(all_trajectories):
            if rospy.is_shutdown():
                break

            traj_msg = Trajectory2D()
            traj_msg.header.stamp = rospy.Time.now()
            traj_msg.header.frame_id = "robot_base_frame" # Or whatever your base link is named

            # Apply the offset to each point from the DXF
            # This makes the trajectory points relative to the robot's actual start position
            for point_relative_to_dxf_origin in raw_traj_points:
                # Add the robot's initial_ee_xy_offset to each DXF point
                # This ensures the DXF pattern starts from the robot's current location
                transformed_point = point_relative_to_dxf_origin + np.array(self.initial_ee_xy_offset)
                
                p = Point()
                p.x = transformed_point[0]
                p.y = transformed_point[1]
                p.z = 0.0 # Z is handled by IK/robot controller
                traj_msg.points.append(p)

            rospy.loginfo(f"Publishing Trajectory {traj_idx + 1} with {len(traj_msg.points)} points.")
            self.trajectory_pub.publish(traj_msg)
            rospy.sleep(0.1) # Small delay between publishing each trajectory to allow subscribers to process

        rospy.loginfo("Finished publishing all DXF trajectories.")
        # This node can optionally shut down or wait for new DXF paths via params.
        # For this example, it publishes once and then waits.
        rospy.signal_shutdown("DXF trajectories published.")


    def run(self):
        rospy.spin() # Keep the node alive


if __name__ == '__main__':
    try:
        node = DXFReaderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass