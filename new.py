import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation

def create_frame(position, quaternion, scale=0.1):
    """Create a coordinate frame at a given position with a given orientation."""
    rot_matrix = Rotation.from_quat(quaternion).as_matrix()
    
    # Create axis lines
    axes = np.eye(3) * scale
    transformed_axes = (rot_matrix @ axes.T).T + position
    
    lines = [[0, 1], [0, 2], [0, 3]]
    colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # RGB for XYZ

    points = [position, *transformed_axes]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)
    
    return line_set

def create_grid(grid_size=1.0, spacing=0.1, axis="xy"):
    """Create a grid centered at the origin."""
    lines = []
    points = []
    
    num_lines = int(grid_size / spacing) + 1
    half_size = grid_size / 2

    for i in range(num_lines):
        coord = -half_size + i * spacing

        if axis == "xy":
            points.append([coord, -half_size, 0])
            points.append([coord, half_size, 0])
            points.append([-half_size, coord, 0])
            points.append([half_size, coord, 0])

        elif axis == "xz":
            points.append([coord, 0, -half_size])
            points.append([coord, 0, half_size])
            points.append([-half_size, 0, coord])
            points.append([half_size, 0, coord])

        elif axis == "yz":
            points.append([0, coord, -half_size])
            points.append([0, coord, half_size])
            points.append([0, -half_size, coord])
            points.append([0, half_size, coord])

    # Create line pairs
    for i in range(0, len(points), 2):
        lines.append([i, i + 1])

    grid = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    
    grid.colors = o3d.utility.Vector3dVector([[0.5, 0.5, 0.5]] * len(lines))  # Gray grid lines
    
    return grid

# Example position and quaternion
position = np.array([-0.495,-0.307,0.055])
quaternion = np.array([ 0.07675785323856764, -3.122286946395002, 1.0561112514591882])  # 90-degree rotation around Z-axis

# Create elements
origin_frame = create_frame(np.array([0, 0, 0]), [0, 0, 0, 1])
target_frame = create_frame(position, quaternion)
grid = create_grid(grid_size=6.0, spacing=0.1, axis="xy")  # XY plane grid

# Visualize
o3d.visualization.draw_geometries([origin_frame, target_frame, grid])


