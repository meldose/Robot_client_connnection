# import numpy as np  
# import matplotlib.pyplot as plt  

# # Function to generate speed graph and success rate graph
# def generate_graphs(t1, t2, robot_velocities, conveyor_speeds, trapezoid_success, pipe_success):
#     time_stamps = np.linspace(t1, t2, len(conveyor_speeds))  # Generate time stamps

#     # Compute robot velocity magnitudes and scale
#     robot_pick_speed = [np.linalg.norm(v) / 1000 for v in robot_velocities]  
    
#     # Create figure with two subplots
#     fig, ax1 = plt.subplots(figsize=(10, 5))
    
#     # First plot: Conveyor & Robot Speed
#     ax1.plot(time_stamps, conveyor_speeds, label='Conveyor Speed (m/s)', color='blue')
#     ax1.plot(time_stamps, robot_pick_speed, label='Robot Pick Speed (m/s)', color='red', linestyle='dashed')
#     ax1.set_xlabel('Time (seconds)')
#     ax1.set_ylabel('Speed (m/s)')
#     ax1.set_title('Speed and Success Rate vs Conveyor Speed')
#     ax1.legend()
#     ax1.grid()

#     # Create second y-axis for success rates
#     ax2 = ax1.twinx()
#     ax2.plot(time_stamps, trapezoid_success, label='Trapezoid Success Rate (%)', color='green', linestyle='dotted')
#     ax2.set_ylabel('Success Rate (%)')
#     ax2.legend(loc='upper right')

#     plt.show()

#     # --- BAR CHART FOR SUCCESS RATE vs CONVEYOR SPEED ---
#     fig, ax = plt.subplots(figsize=(10, 5))

#     # Reduce data points for better visualization (optional)
#     sample_indices = np.linspace(0, len(conveyor_speeds) - 1, 10, dtype=int)
#     sampled_speeds = conveyor_speeds[sample_indices]
#     sampled_trap_success = trapezoid_success[sample_indices]

#     # Bar width and positions
#     bar_width = 0.04  
#     x = sampled_speeds  

#     # Plot bar chart
#     ax.bar(x - bar_width/2, sampled_trap_success, width=bar_width, label='Trapezoid Success Rate', color='green', alpha=0.7)

#     ax.set_xlabel("Conveyor Speed (m/s)")
#     ax.set_ylabel("Success Rate (%)")
#     ax.set_title("Success Rate vs Conveyor Speed")
#     ax.legend()
#     ax.grid(axis='y')

#     plt.show()

# # Example input
# t1, t2 = 1742319295.47, 1742319299.702  
# robot_velocities = [np.array([0.00853, 0.01727, 0]) * 4000 for _ in range(100)]  
# conveyor_speeds = np.linspace(1.0, 2.0, 100)  

# # Simulated success rate trends (assumed to decrease slightly with speed)
# trapezoid_success = np.linspace(90, 85, 100)  # Assume slight drop in success rate
# pipe_success = np.linspace(70, 60, 100)  # Assume bigger drop in success rate

# # Calling the function
# generate_graphs(t1, t2, robot_velocities, conveyor_speeds, trapezoid_success, pipe_success)


#########################################################
import numpy as np
import matplotlib.pyplot as plt

# Example data (Replace with actual values)
avg_pickup_time = np.array([1, 2, 3, 4, 5, 6, 7])  # Time in seconds
success_rate = np.array([80, 85, 88, 90, 92, 94, 95])  # Success Rate in %

# Plot
plt.figure(figsize=(8, 6))
plt.plot(avg_pickup_time, success_rate, marker='o', linestyle='-', color='b', label="Trapezoid")

# Labels and Title
plt.xlabel("Avg Pick Up Time (s)")
plt.ylabel("Success Rate (%)")
plt.title("Avg Pick Up Time vs Success Rate (Trapezoid)")
plt.legend()
plt.grid(True, linestyle="--", alpha=0.7)

# Show Plot
plt.show()
