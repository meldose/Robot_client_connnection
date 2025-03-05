import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

def create_grid(size=5, step=0.5):
    """ Create a 3D grid in Open3D """
    lines = []
    points = []
    
    # X-axis lines
    for i in np.arange(-size, size + step, step):
        points.append([i, -size, 0])  # Start
        points.append([i, size, 0])   # End
        lines.append([len(points) - 2, len(points) - 1])

    # Y-axis lines
    for i in np.arange(-size, size + step, step):
        points.append([-size, i, 0])  # Start
        points.append([size, i, 0])   # End
        lines.append([len(points) - 2, len(points) - 1])

    # Convert to Open3D LineSet
    grid = o3d.geometry.LineSet()
    grid.points = o3d.utility.Vector3dVector(points)
    grid.lines = o3d.utility.Vector2iVector(lines)
    return grid

# Given pose in [x, y, z, qx, qy, qz, qw] format (Quaternion-based)
pose = [-0.495, -0.307, 0.055, 0.03684727, -0.99910336, 0.02085561, 0.01557184] # x, y, z in meters, quaternion (qx, qy, qz, qw)

# Extract position and quaternion
x, y, z, qx, qy, qz, qw = pose

# Convert quaternion (qx, qy, qz, qw) to a rotation matrix
quaternion_rotation_matrix = R.from_quat([qx, qy, qz, qw]).as_matrix()

# Create the original reference frame (fixed at origin)
origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

# Create the transformed coordinate frame (position + quaternion-based rotation)
transformed_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
transformed_frame.rotate(quaternion_rotation_matrix, center=(0, 0, 0))
transformed_frame.translate((x, y, z))

# Create a grid at Z=0 for reference
grid = create_grid(size=5, step=0.5)

# Visualize only the origin frame, transformed frame, and grid
o3d.visualization.draw_geometries([origin_frame, transformed_frame, grid])
