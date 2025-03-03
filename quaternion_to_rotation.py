import numpy as np
from scipy.spatial.transform import Rotation

# Given quaternion in the order [qw, qx, qy, qz]
quaternion_wxyz =  [0.012, -0.519, 0.854, -0.038]  # 90-degree rotation around Y-axis

# Reorder to [qx, qy, qz, qw] for scipy
quaternion_xyzw = [ quaternion_wxyz[0],quaternion_wxyz[1], quaternion_wxyz[2], quaternion_wxyz[3]]

# Convert to rotation matrix
rotation_matrix = Rotation.from_quat(quaternion_xyzw).as_matrix()

print("Rotation Matrix:\n", rotation_matrix)




