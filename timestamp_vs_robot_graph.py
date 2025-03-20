import numpy as np
import matplotlib.pyplot as plt

# Generate timestamps
num_points = 100
time_stamps = np.linspace(0, 100, num_points)  # 100 seconds simulation

# Simulate conveyor speed with some variations
conveyor_speed = np.sin(time_stamps / 10) * 0.5 + 1.5 + np.random.uniform(-0.1, 0.1, num_points)

# Simulate robot picking speed based on conveyor speed (with some delay)
robot_pick_speed = np.maximum(conveyor_speed - np.random.uniform(0.2, 0.5, num_points), 0)

# Plot the graph
plt.figure(figsize=(10, 5))
plt.plot(time_stamps, conveyor_speed, label='Conveyor Speed (m/s)', color='blue')
plt.plot(time_stamps, robot_pick_speed, label='Robot Pick Speed (m/s)', color='red', linestyle='dashed')

plt.xlabel('Time (seconds)')
plt.ylabel('Speed (m/s)')
plt.title('Conveyor and Robot Pick Speed Over Time')
plt.legend()
plt.grid()
plt.show()
