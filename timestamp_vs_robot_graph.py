import numpy as np
import matplotlib.pyplot as plt

# Function to generate the graph
def generate_graph(conveyor_speeds, avg_pick_times):
    # Create figure
    plt.figure(figsize=(10, 5))
    
    # Plot Conveyor Speed vs Average Pick Time
    plt.plot(conveyor_speeds, avg_pick_times, label='Avg Pick Time (s)', color='red', marker='o', linestyle='-')
    
    # Labels and title
    plt.xlabel('Conveyor Speed (m/s)')
    plt.ylabel('Average Pick Time (s)')
    plt.title('Average Pick Time vs Conveyor Speed')
    
    # Legend and Grid
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)

    # Show plot
    plt.show()

# Example input
conveyor_speeds = np.linspace(1.0, 2.0, 100)  # Conveyor speeds in m/s
avg_pick_times = np.linspace(0.5, 1.5, 100)  # Simulated pick times (assumed to increase slightly)

# Calling the function
generate_graph(conveyor_speeds, avg_pick_times)
