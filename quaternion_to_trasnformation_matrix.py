# import numpy as np

# def quaternion_to_rotation_matrix(q):
#     """
#     Convert a quaternion to a 3x3 rotation matrix.
    
#     Parameters:
#     q -- A list or numpy array of quaternion components [w, x, y, z]

#     Returns:
#     Rotation matrix as a 3x3 numpy array
#     """
#     w, x, y, z = q

#     # Rotation matrix components
#     R = np.array([
#         [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
#         [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
#         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
#     ])

#     return R

# def quaternion_to_transformation_matrix(q, translation=[0, 0, 0]):
#     """
#     Convert a quaternion and a translation vector to a 4x4 transformation matrix.

#     Parameters:
#     q -- A list or numpy array of quaternion components [w, x, y, z]
#     translation -- A list or numpy array for the translation vector [tx, ty, tz]

#     Returns:
#     A 4x4 transformation matrix as a numpy array
#     """
#     R = quaternion_to_rotation_matrix(q)
    
#     # Create the 4x4 transformation matrix
#     transformation_matrix = np.eye(4)
#     transformation_matrix[:3, :3] = R
#     transformation_matrix[:3, 3] = translation
    
#     return transformation_matrix

# # Example usage
# q = [0.235,0.063,-0.839,0.486]  # Example quaternion
# translation = [-0.519,-0.362,0.055]      # Example translation
# transformation_matrix = quaternion_to_transformation_matrix(q, translation)

# print("Transformation Matrix:")
# print(transformation_matrix)




import numpy as np
from scipy.spatial.transform import Rotation as R

def transformation_matrix_to_pose(T):
    """
    Convert a 4x4 transformation matrix to a target pose with quaternion rotation and translation.

    Parameters:
    T -- A 4x4 transformation matrix (numpy array)

    Returns:
    rotation_quaternion -- A list or numpy array [w, x, y, z] representing the quaternion
    translation -- A list or numpy array [tx, ty, tz] representing the translation vector
    """
    # Extract rotation matrix (3x3) from the transformation matrix
    rotation_matrix = T[:3, :3]

    # Extract translation vector (3x1) from the transformation matrix
    translation = T[:3, 3]

    # Convert the rotation matrix to a quaternion using scipy's Rotation class
    rotation = R.from_matrix(rotation_matrix)
    rotation_quaternion = rotation.as_quat()  # [x, y, z, w]

    return rotation_quaternion, translation



T = np.array([
 [-0.880234,-0.334134, -0.333094, -0.519   ],
 [ 0.122706 ,0.51967 ,-0.845118, -0.362   ],
 [ 0.455566,-0.785898,-0.41578,0.055   ],
 [ 0,0,0.,1 ]
])  # Example 4x4 transformation matrix

rotation_quaternion, translation = transformation_matrix_to_pose(T)

print("Rotation (Quaternion):", rotation_quaternion)
print("Translation:", translation)
