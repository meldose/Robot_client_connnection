import matplotlib.pyplot as plt
import numpy as np

# Simulation parameters
conveyor_length = 100  # in cm
conveyor_speed = 5      # cm/s
num_objects = 5         # Number of objects on the conveyor
tracking_duration = 20  # seconds

# Generate random starting positions for objects
np.random.seed(42)
object_positions = np.random.randint(0, conveyor_length, num_objects)

# Time array for simulation
time = np.linspace(0, tracking_duration, 200)

# Plot setup
plt.figure(figsize=(12, 6))

# Plot conveyor belt
plt.plot([0, conveyor_length], [0, 0], 'k-', linewidth=4, label='Conveyor Belt')

# Track objects over time
for idx, start_pos in enumerate(object_positions):
    positions = start_pos + conveyor_speed * time
    # Only plot positions within the conveyor length
    valid_indices = positions < conveyor_length
    plt.plot(positions[valid_indices], np.full_like(positions[valid_indices], idx + 1), label=f'Object {idx+1}')

# Graph styling
plt.title('Conveyor Tracking System Using Camera')
plt.xlabel('Conveyor Length (cm)')
plt.ylabel('Object ID')
plt.legend()
plt.grid(True)

plt.show()
