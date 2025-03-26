import numpy as np  
import matplotlib.pyplot as plt  

# Function to generate speed graph and success rate graph
def generate_graphs(t1, t2, robot_velocities, conveyor_speeds):
    time_stamps = np.linspace(t1, t2, len(conveyor_speeds))  # Generate time stamps

    # Compute robot velocity magnitudes and scale
    robot_pick_speed = [np.linalg.norm(v) / 1000 for v in robot_velocities]  
    
    # --- BAR CHART FOR ROBOT TCP SPEED vs CONVEYOR SPEED ---
    fig, ax = plt.subplots(figsize=(10, 5))

    # Reduce data points for better visualization
    sample_indices = np.linspace(0, len(conveyor_speeds) - 1, 10, dtype=int)
    sampled_speeds = conveyor_speeds[sample_indices]
    sampled_robot_speeds = np.array(robot_pick_speed)[sample_indices]

    # Bar width and positions
    bar_width = 0.04  
    x = sampled_speeds  

    # Plot bar chart
    ax.bar(x, sampled_robot_speeds, width=bar_width, color='red', alpha=0.7, label='Robot TCP Speed')

    ax.set_xlabel("Conveyor Speed (m/s)")
    ax.set_ylabel("Robot TCP Speed (m/s)")
    ax.set_title("Robot TCP Speed vs Conveyor Speed")
    ax.legend()
    ax.grid(axis='y')

    plt.show()

# Example input
t1, t2 = 1742319295.47, 1742319299.702  
robot_velocities = [np.array([0.00853, 0.01727, 0]) * 4000 for _ in range(100)]  
conveyor_speeds = np.linspace(1.0, 2.0, 100)  

# Calling the function
generate_graphs(t1, t2, robot_velocities, conveyor_speeds)
