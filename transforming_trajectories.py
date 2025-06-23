import ezdxf
import numpy as np
import matplotlib.pyplot as plt

A4_WIDTH = 210
A4_HEIGHT = 297
MARGIN = 10
USABLE_WIDTH = A4_WIDTH - 2 * MARGIN
USABLE_HEIGHT = A4_HEIGHT - 2 * MARGIN

def remove_close_points(points, min_dist=0.1):
    filtered = [points[0]]
    for pt in points[1:]:
        if np.linalg.norm(pt - filtered[-1]) > min_dist:
            filtered.append(pt)
    return np.array(filtered)


def extract_dxf_trajectories(dxf_path):
    doc = ezdxf.readfile(dxf_path)
    msp = doc.modelspace()
    trajectories = []

    for entity in msp:
        dxftype = entity.dxftype()

        if dxftype == 'LINE':
            start = np.array(entity.dxf.start[:2])
            end = np.array(entity.dxf.end[:2])
            trajectories.append(np.array([start, end]))

        elif dxftype == 'LWPOLYLINE':
            if entity.has_width:
                continue
            points = np.array([[p[0], p[1]] for p in entity.get_points()])
            points = remove_close_points(points)
            trajectories.append(points)

        elif dxftype == 'SPLINE':
            points = np.array(entity.evalpts)
            points = points[:, :2]  # x,y only
            points = remove_close_points(points)
            trajectories.append(points)

        elif dxftype == 'POLYLINE':
            points = np.array([[v.dxf.location.x, v.dxf.location.y] for v in entity.vertices])
            trajectories.append(points)

    return trajectories


def fit_trajectories_to_start(trajectories, desired_start_point, scale_to_a4=True):
    all_points = np.concatenate(trajectories)
    min_pt = all_points.min(axis=0)
    max_pt = all_points.max(axis=0)
    size = max_pt - min_pt

    scale = 1.0
    if scale_to_a4:
        scale_x = USABLE_WIDTH / size[0]
        scale_y = USABLE_HEIGHT / size[1]
        scale = min(scale_x, scale_y) * 0.9

    # Shift all points so min_pt is at origin and scale
    shifted_scaled_points = [(traj - min_pt) * scale for traj in trajectories]

    # Now flip Y axis: Y goes from 0 (bottom) to USABLE_HEIGHT (top)
    # Because after shift min_pt is at zero, Y values go from 0 to scaled_height
    scaled_height = size[1] * scale

    transformed = []
    for traj in shifted_scaled_points:
        flipped_y = scaled_height - traj[:,1]  # flip y around scaled height

        # Now translate so bottom-left corner matches desired_start_point
        aligned_x = traj[:,0] + desired_start_point[0]
        aligned_y = flipped_y + desired_start_point[1]

        aligned = np.column_stack((aligned_x, aligned_y))
        transformed.append(aligned)

    return transformed, scale




def get_starting_point(trajectories, fallback_value=None):
    if fallback_value is not None:
        return np.array(fallback_value)
    return trajectories[0][0]


def plot_trajectories(trajectories, start_point=None):
    fig_width_in = A4_WIDTH/25.4
    fig_height_in = A4_HEIGHT/25.4
    plt.figure(figsize=(fig_width_in, fig_height_in))

    ax = plt.gca()
    for traj in trajectories:
        traj = np.asarray(traj)
        if traj.ndim != 2 or traj.shape[1] != 2:
            continue
        ax.plot(traj[:, 0], traj[:, 1], '-', color='blue')

    if start_point is not None:
        ax.plot(start_point[0], start_point[1], 'ro', markersize=5, label="Start")
        # Manual scale font size by data units (e.g., 5 mm)
        font_size_in_mm = 5
        # Convert mm to display points roughly (assuming 72 points/inch and 25.4 mm/inch)
        # pts = mm * (72 / 25.4)
        fontsize_pts = font_size_in_mm * (72 / 25.4)
        ax.text(start_point[0] + 2, start_point[1] + 2, "Start", fontsize=fontsize_pts, color='red')

        ax.legend()

    ax.set_aspect('equal')
    ax.set_title("Trajectories Anchored at Start Point")
    ax.set_xlim(0, A4_WIDTH)
    ax.set_ylim(0, A4_HEIGHT)
    ax.invert_yaxis()
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.grid(True)
    plt.show()




if __name__ == "__main__":
    dxf_file = "Pain.dxf"  # make sure this file exists
    trajectories = extract_dxf_trajectories(dxf_file)

    desired_start = np.array([MARGIN, MARGIN])
    transformed_trajectories, _ = fit_trajectories_to_start(trajectories, desired_start)

    plot_trajectories(transformed_trajectories, start_point=desired_start)
