import numpy as np
from scipy.spatial.transform import Rotation

# Define the rotation matrix
rotation_matrix = np.array([
 [-0.99682484 ,0.0524341  ,0.05992412],
 [-0.07733951, -0.45853349, -0.88530539],
 [-0.01894298, -0.88712891, 0.4611328 ]])

# Convert to Euler angles (XYZ convention)
rotation = Rotation.from_matrix(rotation_matrix)
euler_angles = rotation.as_euler('xyz', degrees=False)  # Convert to degrees

# Print Euler angles
print(f"Roll (X): {euler_angles[0]:.2f}°")
print(f"Pitch (Y): {euler_angles[1]:.2f}°")
print(f"Yaw (Z): {euler_angles[2]:.2f}°")
