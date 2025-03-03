import numpy as np
from scipy.spatial.transform import Rotation

# Given quaternion (qx, qy, qz, qw)
quaternion = [0.012, -0.519, 0.854, -0.038]

# Convert quaternion to rotation matrix
rotation = Rotation.from_quat(quaternion)
rotation_matrix = rotation.as_matrix()

print(rotation_matrix)
