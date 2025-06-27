import rospy
import math
import numpy as np
import sys
import os
import tty
import termios
import time
import threading
from typing import Tuple, Optional
MATPLOTLIB_AVAILABLE = False

EZDXF_AVAILABLE = False
try:
    import ezdxf
    EZDXF_AVAILABLE = True
except ImportError:
    print("The 'ezdxf' library is required for DXF parsing. Please run: pip install ezdxf")

try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    print("The 'matplotlib' library is recommended for trajectory visualization. Please run: pip install matplotlib")

# --- ROS-related imports ---
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def plot_trajectories(trajectories, offset=[0,0]):
    if not MATPLOTLIB_AVAILABLE:
        rospy.logwarn("Matplotlib not found. Skipping trajectory plot.")
        return
    plt.figure(figsize=(10, 8))
    ax = plt.gca()
    for i, traj in enumerate(trajectories):
        offset_traj = traj + np.array(offset)
        ax.plot(offset_traj[:, 0], offset_traj[:, 1], marker='.')
    ax.set_aspect('equal', adjustable='box')
    plt.title(f'DXF Trajectories Preview (Offset by {np.round(offset, 3)})')
    plt.xlabel('X coordinate (m)')
    plt.ylabel('Y coordinate (m)')
    plt.grid(True)
    plt.show(block=False)
    plt.pause(2)
    rospy.loginfo("Trajectory plot displayed. Continuing execution...")

def extract_trajectories_from_dxf_in_meters(filepath, curve_sampling_tolerance=0.001, straight_segment_max_length_m=0.001):
    """
    Extracts trajectories from a DXF file, including more entity types, and normalizes their positions.
    This version also adds more points to straight line segments.

    This function reads a DXF file, finds various drawing entities (LWPOLYLINE, POLYLINE,
    LINE, ARC, CIRCLE, ELLIPSE, SPLINE, and explodes INSERT entities),
    and converts their coordinates from millimeters to meters. It samples curved
    entities to generate a sequence of points and interpolates points along
    straight line segments to ensure a minimum point density.
    It then takes the starting point of the *first* trajectory found and uses it
    as a global origin. All trajectories are then translated by this same offset,
    ensuring the first trajectory starts at (0,0) and all others maintain their
    relative position to it.

    Args:
        filepath (str): The path to the DXF file.
        curve_sampling_tolerance (float): A tolerance value for sampling curved
                                          entities (e.g., arcs, circles, splines).
                                          Smaller values result in more points.
                                          Value in original DXF drawing units (mm).
        straight_segment_max_length_m (float): The maximum desired length in meters
                                               between points on a straight line segment.
                                               If a segment is longer, points will be
                                               interpolated.

    Returns:
        list: A list of numpy arrays, where each array represents a trajectory
              normalized relative to the start of the first trajectory.
              Returns an empty list if no trajectories are found or an error occurs.
    """
    if not EZDXF_AVAILABLE:
        rospy.logerr("ezdxf library not available. Cannot extract trajectories from DXF.")
        return []

    raw_trajectories = []

    # Helper function to interpolate points on a straight line segment
    def interpolate_segment(p1_meters, p2_meters, max_length_m):
        segment_points = [p1_meters]
        p1 = np.array(p1_meters)
        p2 = np.array(p2_meters)
        
        distance = np.linalg.norm(p2 - p1)
        if distance > max_length_m:
            num_segments = int(np.ceil(distance / max_length_m))
            for i in range(1, num_segments):
                ratio = float(i) / num_segments
                interpolated_point = p1 + ratio * (p2 - p1)
                segment_points.append(interpolated_point.tolist())
        segment_points.append(p2_meters)
        return segment_points

    try:
        doc = ezdxf.readfile(filepath)
        msp = doc.modelspace()

        exploded_entities = []
        for entity in msp:
            if entity.dxftype() == 'INSERT':
                exploded_entities.extend(list(entity.explode()))
            else:
                exploded_entities.append(entity)

        for entity in exploded_entities:
            current_entity_points = []
            dxftype = entity.dxftype()

            if dxftype == 'LWPOLYLINE':
                # get_points can return a list of (x, y, start_width, end_width, bulge)
                # or just (x, y) if format='xy'. We need to process segments.
                # If curve_accuracy is supported, use it. Otherwise, rely on default.
                try:
                    raw_points_mm = entity.get_points(format='xy', curve_accuracy=curve_sampling_tolerance)
                except TypeError: # Older ezdxf versions don't have curve_accuracy
                    raw_points_mm = entity.get_points(format='xy')
                
                if not raw_points_mm:
                    continue

                # Convert all points to meters first
                points_in_meters = [[p[0] / 1000.0, p[1] / 1000.0] for p in raw_points_mm]
                
                # Iterate through segments for interpolation
                current_entity_points.append(points_in_meters[0]) # Add first point
                for i in range(len(points_in_meters) - 1):
                    p1 = points_in_meters[i]
                    p2 = points_in_meters[i+1]
                    interpolated_seg_points = interpolate_segment(p1, p2, straight_segment_max_length_m)
                    # Add all interpolated points except the very first one (which is p1, already added or is part of prev segment's end)
                    current_entity_points.extend(interpolated_seg_points[1:])


            elif dxftype == 'POLYLINE':
                # Similar to LWPOLYLINE, convert vertices to meters and then interpolate
                raw_vertices = [vertex.dxf.location for vertex in entity.vertices]
                if not raw_vertices:
                    continue

                points_in_meters = [[v.x / 1000.0, v.y / 1000.0] for v in raw_vertices]

                current_entity_points.append(points_in_meters[0]) # Add first point
                for i in range(len(points_in_meters) - 1):
                    p1 = points_in_meters[i]
                    p2 = points_in_meters[i+1]
                    interpolated_seg_points = interpolate_segment(p1, p2, straight_segment_max_length_m)
                    current_entity_points.extend(interpolated_seg_points[1:])


            elif dxftype == 'LINE':
                start_m = [entity.dxf.start.x / 1000.0, entity.dxf.start.y / 1000.0]
                end_m = [entity.dxf.end.x / 1000.0, entity.dxf.end.y / 1000.0]
                current_entity_points.extend(interpolate_segment(start_m, end_m, straight_segment_max_length_m))

            elif dxftype == 'ARC':
                # Use flattening, it handles sampling based on tolerance
                for p in entity.flattening(curve_sampling_tolerance):
                    current_entity_points.append([p.x / 1000.0, p.y / 1000.0])

            elif dxftype == 'CIRCLE':
                # Use flattening, it handles sampling based on tolerance
                for p in entity.flattening(curve_sampling_tolerance):
                    current_entity_points.append([p.x / 1000.0, p.y / 1000.0])

            elif dxftype == 'ELLIPSE':
                # Use flattening, it handles sampling based on tolerance
                for p in entity.flattening(curve_sampling_tolerance):
                    current_entity_points.append([p.x / 1000.0, p.y / 1000.0])

            elif dxftype == 'SPLINE':
                # Use flattening, it handles sampling based on tolerance
                for p in entity.flattening(curve_sampling_tolerance):
                    current_entity_points.append([p.x / 1000.0, p.y / 1000.0])

            if current_entity_points:
                # Filter out duplicate consecutive points
                if len(current_entity_points) > 1:
                    unique_points = [current_entity_points[0]]
                    for i in range(1, len(current_entity_points)):
                        if not np.allclose(current_entity_points[i], unique_points[-1]): # Compare to last unique point
                            unique_points.append(current_entity_points[i])
                    current_entity_points = unique_points
                
                if len(current_entity_points) > 0: # Ensure we still have points after de-duplication
                    raw_trajectories.append(np.array(current_entity_points))

    except IOError:
        rospy.logerr(f"Error: Could not find or open the DXF file at '{filepath}'")
        return []
    except ezdxf.DXFStructureError:
        rospy.logerr(f"Error: The DXF file at '{filepath}' is invalid or corrupt.")
        return []
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred while processing DXF: {e}")
        return []

    if not raw_trajectories:
        rospy.logwarn(f"No valid trajectories found in {filepath}.")
        return []

    # Ensure the first trajectory actually has points before trying to access its first element
    if not raw_trajectories[0].size > 0:
        rospy.logerr(f"First trajectory found is empty in {filepath}. Cannot normalize.")
        return []

    global_offset = raw_trajectories[0][0]
    normalized_trajectories = [traj - global_offset for traj in raw_trajectories]

    rospy.loginfo(f"Successfully extracted and normalized {len(normalized_trajectories)} trajectories from {filepath}.")
    return normalized_trajectories