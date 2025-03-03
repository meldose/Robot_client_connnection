import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

def plot_euler_coordinates(coords):
    if len(coords) != 6:
        raise ValueError("Input must be a list or tuple of 6 elements: [x, y, z, roll, pitch, yaw]")
    
    x, y, z, roll, pitch, yaw = coords
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Convert Euler angles to rotation matrix
    r = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
    rotation_matrix = r.as_matrix()
    
    # Define coordinate frame unit vectors
    origin = np.array([x, y, z])
    x_axis = origin + rotation_matrix[:, 0]  # X direction
    y_axis = origin + rotation_matrix[:, 1]  # Y direction
    z_axis = origin + rotation_matrix[:, 2]  # Z direction
    
    # Plot the origin
    ax.scatter(x, y, z, color='black', label='Origin')
    
    # Plot coordinate frame vectors
    ax.quiver(x, y, z, x_axis[0] - x, x_axis[1] - y, x_axis[2] - z, color='r', label='X-axis')
    ax.quiver(x, y, z, y_axis[0] - x, y_axis[1] - y, y_axis[2] - z, color='g', label='Y-axis')
    ax.quiver(x, y, z, z_axis[0] - x, z_axis[1] - y, z_axis[2] - z, color='b', label='Z-axis')
    
    # Labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Euler Coordinate Visualization')
    ax.legend()
    
    plt.show()

# Example input
plot_euler_coordinates([-0.495, -0.307, 0.055, 0.07675785323856764, -3.122286946395002, 1.0561112514591882])
