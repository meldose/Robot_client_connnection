import numpy as np
import matplotlib.pyplot as plt

# Function to generate speed graph with variations
def generate_graphs_with_variations(t1, t2, robot_velocities, conveyor_speeds):
    time_stamps = np.linspace(t1, t2, len(conveyor_speeds))  # Generate time stamps

    # Compute robot velocity magnitudes and scale (with noise and non-linearity)
    noise_factor = 0.1  # 10% noise
    robot_pick_speed = [np.linalg.norm(np.array(v) + np.random.normal(0, noise_factor, 3)) / 1000 for v in robot_velocities]
    
    # --- LINE CHART FOR ROBOT TCP SPEED vs CONVEYOR SPEED ---
    fig, ax = plt.subplots(figsize=(10, 5))

    # Reduce data points for better visualization
    sample_indices = np.linspace(0, len(conveyor_speeds) - 1, 10, dtype=int)
    sampled_speeds = conveyor_speeds[sample_indices]
    sampled_robot_speeds = np.array(robot_pick_speed)[sample_indices]

    # Plot line graph with markers
    ax.plot(sampled_speeds, sampled_robot_speeds, linestyle='-', marker='o', color='red', label='Robot TCP Speed')

    # Labels and title
    ax.set_xlabel("Conveyor Speed (m/s)")
    ax.set_ylabel("Robot TCP Speed (m/s)")
    ax.set_title("Robot TCP Speed vs Conveyor Speed with Variations")
    
    ax.legend()
    ax.grid(True, linestyle='--', alpha=0.5)

    # Improve x-axis ticks
    ax.set_xticks(sampled_speeds)
    ax.set_xticklabels([f"{s:.2f}" for s in sampled_speeds], rotation=45)

    plt.show()

# Example input
t1, t2 = 1742319295.47, 1742319299.702  
conveyor_speeds = np.linspace(1.0, 2.0, 100)  

# Non-linear robot velocities with noise
robot_velocities = [np.array([0.00853, 0.01727, 0]) * (4000 * s**2) for s in conveyor_speeds]  

# Calling the function
generate_graphs_with_variations(t1, t2, robot_velocities, conveyor_speeds)
