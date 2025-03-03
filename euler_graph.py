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

# Given pose in [x, y, z, Roll, Pitch, Yaw] format
pose = [-0.495, -0.307, 0.055, 0.07675785323856764, -3.122286946395002, 1.0561112514591882] # x, y, z in meters, angles in degrees

# Extract position and orientation
x, y, z, roll, pitch, yaw = pose

# Convert Euler angles (Roll=X, Pitch=Y, Yaw=Z) to a rotation matrix
rotation_matrix = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix()

# Create the original reference frame (fixed at origin)
origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

# Create the transformed coordinate frame
transformed_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

# Apply rotation and translation to the transformed frame
transformed_frame.rotate(rotation_matrix, center=(0, 0, 0))
transformed_frame.translate((x, y, z))

# Create a grid at Z=0 for reference
grid = create_grid(size=5, step=0.5)

# Visualize
o3d.visualization.draw_geometries([origin_frame, transformed_frame, grid])
