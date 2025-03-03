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

def create_axis_labels(size=0.1):
    """ Create small spheres to label X, Y, and Z axes """
    labels = []
    
    # X-axis label (Red)
    x_label = o3d.geometry.TriangleMesh.create_sphere(radius=size)
    x_label.paint_uniform_color([1, 0, 0])  # Red
    x_label.translate((0.5, 0, 0))  # Position at +X
    
    # Y-axis label (Green)
    y_label = o3d.geometry.TriangleMesh.create_sphere(radius=size)
    y_label.paint_uniform_color([0, 1, 0])  # Green
    y_label.translate((0, 0.5, 0))  # Position at +Y
    
    # Z-axis label (Blue)
    z_label = o3d.geometry.TriangleMesh.create_sphere(radius=size)
    z_label.paint_uniform_color([0, 0, 1])  # Blue
    z_label.translate((0, 0, 0.5))  # Position at +Z

    return [x_label, y_label, z_label]

# Given pose in [x, y, z, qx, qy, qz, qw] format (Quaternion-based)
pose = [-0.492, -0.296, 0.055, 0.012, -0.519, 0.854, -0.038] # x, y, z in meters, quaternion (qx, qy, qz, qw)

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

# Create the quaternion-based coordinate frame at the origin (for reference)
quaternion_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
quaternion_frame.rotate(quaternion_rotation_matrix, center=(0, 0, 0))  # Only rotation, no translation

# Create a grid at Z=0 for reference
grid = create_grid(size=5, step=0.5)

# Create labeled axis markers
axis_labels = create_axis_labels(size=0.05)

# Visualize
o3d.visualization.draw_geometries([origin_frame, transformed_frame, quaternion_frame, grid] + axis_labels)
