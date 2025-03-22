import numpy as np # importing numpy as np
import matplotlib.pyplot as plt # importing matplotlib.pyplot as plt

# Function to generate speed graph
def generate_speed_graph(change_in_time, robot_velocities, conveyor_speeds): # defining the function
    time_stamps = np.linspace(t1, t2, len(conveyor_speeds)) # setting the time stamps
    

    robot_pick_speed = [np.linalg.norm(v) for v in robot_velocities] # Extract robot velocity magnitudes
    
    # Plot the graph
    plt.figure(figsize=(10, 5))
    plt.plot(time_stamps, conveyor_speeds, label='Conveyor Speed (m/s)', color='blue')
    plt.plot(time_stamps, robot_pick_speed, label='Robot Pick Speed (m/s)', color='red', linestyle='dashed')
    
    plt.xlabel('Time (seconds)') # plotting the x axis
    plt.ylabel('Speed (m/s)') # plotting the y axis
    plt.title('Conveyor and Robot Pick Speed Over Time') # plotting the title
    plt.legend() # plotting the legend
    plt.grid() # plotting the grid
    plt.show() # plotting the graph

# Example input
t1, t2 = 1742319295.47,1742319299.702 # setting the time t1 and t2
change_in_time= t2-t1 # change in time
robot_velocities = [np.array([0.00853,0.01727,0])*6500 for _ in range(100)]  # Example constant velocities
conveyor_speeds = np.linspace(1.0, 2.0, 100)  # Example conveyor speeds

generate_speed_graph(change_in_time, robot_velocities, conveyor_speeds) # calling the function
