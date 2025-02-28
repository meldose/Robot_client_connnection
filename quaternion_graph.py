import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

def plot_quaternion(data, ax=None):
    """Visualize a quaternion as a rotated coordinate frame."""
    X, Y, Z, w, a, b, c = data  # Extract values
    q = [a, b, c, w]  # Rearrange quaternion elements to (x, y, z, w)
    
    if ax is None:
        fig = plt.figure(figsize=(6, 6))
        ax = fig.add_subplot(111, projection='3d')
    
    # Convert quaternion to rotation matrix
    r = R.from_quat(q)
    rot_matrix = r.as_matrix()
    
    # Define origin and axes
    origin = np.array([X, Y, Z])
    colors = ['r', 'g', 'b']  # X, Y, Z axis colors
    labels = ['X', 'Y', 'Z']
    
    # Plot the rotated coordinate frame
    for i in range(3):
        vec = rot_matrix[:, i]  # Get column of rotation matrix
        ax.quiver(*origin, *vec, color=colors[i], length=1.0)
        ax.text(*(origin + vec), labels[i], color=colors[i], fontsize=12)
    
    # Set limits and labels
    ax.set_xlim([X - 1, X + 1])
    ax.set_ylim([Y - 1, Y + 1])
    ax.set_zlim([Z - 1, Z + 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title("Quaternion Rotation Visualization")
    plt.show()

# Example input [X, Y, Z, w, a, b, c, d] (where w, a, b, c form the quaternion)
example_input = [-0.495,-0.307,0.055,0.011,-0.504,0.863,-0.038]  # 90-degree rotation around Z-axis
plot_quaternion(example_input)
