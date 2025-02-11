import matplotlib.pyplot as plt # importing matplotlib
import numpy as np # importing numpy

# Simulation parameters
conveyor_length = 100  # in cm
conveyor_speed = 5      # cm/s
num_objects = 5         # Number of objects on the conveyor
tracking_duration = 20  # seconds

# Generate random starting positions for objects
np.random.seed(42)  # for reproducibility
object_positions = np.random.randint(0, conveyor_length, num_objects) # cm

# Time array for simulation
time = np.linspace(0, tracking_duration, 200) # seconds

# Plot setup
plt.figure(figsize=(12, 6)) # set figure size

# Plot conveyor belt
plt.plot([0, conveyor_length], [0, 0], 'k-', linewidth=4, label='Conveyor Belt') # plot the conveyor belt

# Track objects over time
for idx, start_pos in enumerate(object_positions): # for each object
    positions = start_pos + conveyor_speed * time # cm
    # Only plot positions within the conveyor length
    valid_indices = positions < conveyor_length # cm
    plt.plot(positions[valid_indices], np.full_like(positions[valid_indices], idx + 1), label=f'Object {idx+1}') # plot the object

# Graph styling
plt.title('Conveyor Tracking System Using Camera') # set title
plt.xlabel('Conveyor Length (cm)') # set x-axis label
plt.ylabel('Object ID') # set y-axis label
plt.legend() # show legend
plt.grid(True) # show grid

plt.show() # show the plot
