import numpy as np # importing numpy
import matplotlib.pyplot as plt # importing matplotlib

# Function to generate speed graph and success rate graph
def generate_graphs(t1, t2, robot_velocities, conveyor_speeds, trapezoid_success, pipe_success):
    time_stamps = np.linspace(t1, t2, len(conveyor_speeds))  # Generate time stamps

    # Compute robot velocity magnitudes and scale
    robot_pick_speed = [np.linalg.norm(v) / 1000 for v in robot_velocities]
    
    # Create figure with two subplots
    fig, ax1 = plt.subplots(figsize=(10, 5))
    
    # First plot: Conveyor & Robot Speed
    ax1.plot(time_stamps, conveyor_speeds, label='Conveyor Speed (m/s)', color='blue') # plot the conveyor speed
    ax1.plot(time_stamps, robot_pick_speed, label='Robot Pick Speed (m/s)', color='red', linestyle='dashed') # plot the robot pick speed
    ax1.set_xlabel('Time (seconds)') # x-axis label
    ax1.set_ylabel('Speed (m/s)') # y-axis label
    ax1.set_title('Speed and Success Rate vs Conveyor Speed')# title
    ax1.legend() # show legend
    ax1.grid() # show grid

    # Create second y-axis for success rates
    ax2 = ax1.twinx() # share x-axis
    ax2.plot(time_stamps, trapezoid_success, label='Trapezoid Success Rate (%)', color='green', linestyle='dotted') # plot the trapezoid success rate
    ax2.plot(time_stamps, pipe_success, label='Pipe Success Rate (%)', color='purple', linestyle='dashdot') # plot the pipe success rate
    ax2.set_ylabel('Success Rate (%)') # y-axis label
    ax2.legend(loc='upper right') # legend location

    plt.show() # show the plot

# Example input
t1, t2 = 1742319295.47, 1742319299.702 # seconds
robot_velocities = [np.array([0.00853, 0.01727, 0]) * 4000 for _ in range(100)]  # Generate robot velocities 
conveyor_speeds = np.linspace(1.0, 2.0, 100) # Generate conveyor speeds

# Simulated success rate trends (assumed to decrease slightly with speed)
trapezoid_success = np.linspace(90, 85, 100)  # Assume slight drop in success rate
pipe_success = np.linspace(70, 60, 100)  # Assume bigger drop in success rate

# Calling the function
generate_graphs(t1, t2, robot_velocities, conveyor_speeds, trapezoid_success, pipe_success)
