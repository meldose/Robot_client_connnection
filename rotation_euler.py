import numpy as np
from scipy.spatial.transform import Rotation

# Define the rotation matrix
rotation_matrix = np.array([
    [-0.9968,  0.0524,  0.0599],
    [-0.0773, -0.4585, -0.8853],
    [-0.0189, -0.8871,  0.4611]
])

# Convert to Euler angles (XYZ convention)
rotation = Rotation.from_matrix(rotation_matrix)
euler_angles = rotation.as_euler('xyz', degrees=False)  # Convert to degrees

# Print Euler angles
print(f"Roll (X): {euler_angles[0]:.2f}°")
print(f"Pitch (Y): {euler_angles[1]:.2f}°")
print(f"Yaw (Z): {euler_angles[2]:.2f}°")
