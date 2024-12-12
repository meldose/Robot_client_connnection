import numpy as np

def quaternion_to_rotation_matrix(q):
    """
    Convert a quaternion to a 3x3 rotation matrix.
    
    Parameters:
    q -- A list or numpy array of quaternion components [w, x, y, z]

    Returns:
    Rotation matrix as a 3x3 numpy array
    """
    w, x, y, z = q

    # Rotation matrix components
    R = np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
    ])

    return R

def quaternion_to_transformation_matrix(q, translation=[0, 0, 0]):
    """
    Convert a quaternion and a translation vector to a 4x4 transformation matrix.

    Parameters:
    q -- A list or numpy array of quaternion components [w, x, y, z]
    translation -- A list or numpy array for the translation vector [tx, ty, tz]

    Returns:
    A 4x4 transformation matrix as a numpy array
    """
    R = quaternion_to_rotation_matrix(q)
    
    # Create the 4x4 transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = R
    transformation_matrix[:3, 3] = translation
    
    return transformation_matrix

# Example usage
q = [0.235,0.063,-0.839,0.486]  # Example quaternion
translation = [-0.519,-0.362,0.055]      # Example translation
transformation_matrix = quaternion_to_transformation_matrix(q, translation)

print("Transformation Matrix:")
print(transformation_matrix)



