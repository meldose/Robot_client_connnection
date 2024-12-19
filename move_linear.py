    

# Method to do Servo in Cartesian Space

# Args:
# - servo_target_position ([int]*7)     : List of cartesian target position in m. Length of list should be equal to 7 (X, Y, Z, qw, qx, qz, qz)
# - servo_target_velocity ([int]*7)     : List of cartesian target velocity in m/s. Length of list should be equal to 7
# - servo_target_acceleration ([int]*7) : List of cartesian target acceleration in m/s^2. Length of list should be equal 7. Not used in Current Version
# - gain parameter (double)             : ServoX gain parameter, used for Propotional controller (should be between [0.2, 100])

# from neurapy.robot import Robot
# import time
# from ruckig import InputParameter, OutputParameter, Result, Ruckig
# import copy

# r = Robot()
# r.activate_servo_interface('position')

# def movelinear_online():
#     cart_pose_length = 7 #X,Y,Z,qw,qx,qy,qz

#     otg = Ruckig(cart_pose_length, 0.001)  # control cycle
#     inp = InputParameter(cart_pose_length)
#     out = OutputParameter(cart_pose_length)

#     inp.current_position = r.get_current_cartesian_pose()
#     inp.current_velocity = [0.]*cart_pose_length
#     inp.current_acceleration = [0.]*cart_pose_length

#     target = copy.deepcopy(inp.current_position)
#     target[0] += 0.2 # Move 200mm in X direction
#     inp.target_position = target
#     inp.target_velocity = [0.]*cart_pose_length
#     inp.target_acceleration = [0.]*cart_pose_length

#     inp.max_velocity = [0.5]*cart_pose_length
#     inp.max_acceleration = [3]*cart_pose_length
#     inp.max_jerk = [10.]*cart_pose_length
#     res = Result.Working

#     servox_proportional_gain = 25

#     while res == Result.Working:
#         '''
#         Error code is returned through Servo. 
#         '''
#         error_code = 0
#         if(error_code < 3):

#             res = otg.update(inp, out)

#             position = out.new_position
#             velocity = out.new_velocity 
#             acceleration = out.new_acceleration

#             error_code = r.servo_x(position, velocity, acceleration, servox_proportional_gain)
#             scaling_factor = r.get_servo_trajectory_scaling_factor()
#             out.pass_to_input(inp)
#             time.sleep(0.001)
#         else:
#             print("Servo in error, error code, ", error_code)
#             break
#     r.deactivate_servo_interface()

#     r.stop()

    
#     # self.logger.info(
#     #         "MOVELINEAR called with parameters {} {}".format(args, kwargs)
#     #     )
#     # if("maira" in self.robot_name.lower()):
#     #     self.robot.logger.warning("MOVELINEAR not implemented for Maira. Stay tuned for updates.")
#     #     return False
    
#     # command = Servo(self)
#     # command.execute_visual_servoing(*args,**kwargs)

# movelinear_online()


import time
from neurapy.robot import Robot
from ruckig import InputParameter, OutputParameter, Result, Ruckig
from scipy.spatial.transform import Rotation as R  # Import Rotation

def quaternion_to_rpy(qw, qx, qy, qz, degrees=False):
    """
    Convert a quaternion into Roll, Pitch, and Yaw angles.

    Args:
        qw, qx, qy, qz (float): Quaternion components.
        degrees (bool): If True, return angles in degrees. Otherwise, in radians.

    Returns:
        tuple: (roll, pitch, yaw) angles.
    """
    # Normalize the quaternion to ensure it's valid
    norm = (qw**2 + qx**2 + qy**2 + qz**2) ** 0.5
    if norm == 0:
        raise ValueError("Zero-norm quaternion is invalid for conversion.")
    qw_norm = qw / norm
    qx_norm = qx / norm
    qy_norm = qy / norm
    qz_norm = qz / norm

    rotation = R.from_quat([qx_norm, qy_norm, qz_norm, qw_norm])  # Note the order: x, y, z, w
    rpy = rotation.as_euler('xyz', degrees=degrees)
    return rpy  # Returns a tuple (roll, pitch, yaw)

def servo_cartesian(
    servo_target_position, 
    servo_target_velocity, 
    servo_target_acceleration, 
    gain
):
    """
    Perform servo control in Cartesian space.

    Args:
        servo_target_position (list of float): Cartesian target positions [X, Y, Z, qw, qx, qy, qz] in meters. Length must be 7.
        servo_target_velocity (list of float): Cartesian target velocities [X, Y, Z, qw, qx, qy, qz] in m/s. Length must be 7.
        servo_target_acceleration (list of float): Cartesian target accelerations [X, Y, Z, qw, qx, qy, qz] in m/s². Length must be 7. Not used currently.
        gain (float): ServoX proportional gain parameter. Should be between 0.2 and 100.

    Returns:
        bool: True if servo control completed successfully, False otherwise.
    """
    
    # Validate input lengths
    if not (len(servo_target_position) == len(servo_target_velocity) == len(servo_target_acceleration) == 7):
        raise ValueError("All input lists must have exactly 7 elements [X, Y, Z, qw, qx, qy, qz].")
    
    # Validate gain
    if not (0.2 <= gain <= 100):
        raise ValueError("Gain must be between 0.2 and 100.")
    
    # Initialize robot and activate servo interface
    robot = Robot()
    robot.activate_servo_interface('position')
    
    # Define parameters
    cartesian_dims = 7  # X, Y, Z, qw, qx, qy, qz
    control_cycle = 0.001  # 1 ms control cycle
    
    # Initialize Ruckig
    ruckig = Ruckig(cartesian_dims, control_cycle)
    input_param = InputParameter(cartesian_dims)
    output_param = OutputParameter(cartesian_dims)
    
    # Set current state
    current_pose = robot.get_current_cartesian_pose()
    input_param.current_position = current_pose
    input_param.current_velocity = [0.0] * cartesian_dims
    input_param.current_acceleration = [0.0] * cartesian_dims
    
    # Convert current quaternion to RPY for logging
    current_rpy = quaternion_to_rpy(current_pose[3], current_pose[4], current_pose[5], current_pose[6], degrees=True)
    print(f"Current Pose (RPY): Roll={current_rpy[0]:.2f}°, Pitch={current_rpy[1]:.2f}°, Yaw={current_rpy[2]:.2f}°")
    
    # Set target state
    input_param.target_position = servo_target_position
    input_param.target_velocity = servo_target_velocity
    input_param.target_acceleration = servo_target_acceleration  # Not used currently
    
    # Convert target quaternion to RPY for logging
    target_rpy = quaternion_to_rpy(
        servo_target_position[3], 
        servo_target_position[4], 
        servo_target_position[5], 
        servo_target_position[6], 
        degrees=True
    )
    print(f"Target Pose (RPY): Roll={target_rpy[0]:.2f}°, Pitch={target_rpy[1]:.2f}°, Yaw={target_rpy[2]:.2f}°")
    
    # Set constraints
    input_param.max_velocity = [0.5] * cartesian_dims
    input_param.max_acceleration = [3.0] * cartesian_dims
    input_param.max_jerk = [10.0] * cartesian_dims
    
    # Initialize result
    result = Result.Working
    
    # Servo control loop
    while result == Result.Working:
        # Update trajectory
        result = ruckig.update(input_param, output_param)
        
        if result == Result.Working:
            # Extract new trajectory points
            new_position = output_param.new_position
            new_velocity = output_param.new_velocity
            new_acceleration = output_param.new_acceleration
            
            # Convert new quaternion to RPY for logging
            new_rpy = quaternion_to_rpy(new_position[3], new_position[4], new_position[5], new_position[6], degrees=True)
            print(f"New Trajectory Pose (RPY): Roll={new_rpy[0]:.2f}°, Pitch={new_rpy[1]:.2f}°, Yaw={new_rpy[2]:.2f}°")
            
            # Send servo command
            error_code = robot.servo_x(new_position, new_velocity, new_acceleration, gain)
            
            # Check for servo errors
            if error_code >= 3:
                print(f"Servo error encountered. Error code: {error_code}")
                break
            
            # Optionally handle scaling factor
            scaling_factor = robot.get_servo_trajectory_scaling_factor()
            # You can use scaling_factor if needed
            
            # Prepare for next iteration
            input_param.current_position = new_position[:]  # Shallow copy is sufficient
            input_param.current_velocity = new_velocity[:]
            input_param.current_acceleration = new_acceleration[:]
            
            # Wait for next control cycle
            time.sleep(control_cycle)
        else:
            if result == Result.Finished:
                print("Trajectory execution completed successfully.")
            else:
                print(f"Ruckig update failed with result: {result}")
            break
    
    # Deactivate servo interface and stop robot
    robot.deactivate_servo_interface()
    robot.stop()
    
    # Return success status
    return result == Result.Finished

# Example usage
if __name__ == "__main__":
    from scipy.spatial.transform import Rotation as R
    
    # Desired RPY angles in degrees for target orientation
    desired_roll = 30    # degrees
    desired_pitch = 45   # degrees
    desired_yaw = 60     # degrees
    
    # Create a Rotation object from RPY angles
    rotation = R.from_euler('xyz', [desired_roll, desired_pitch, desired_yaw], degrees=True)
    
    # Extract quaternion components (scipy returns [x, y, z, w])
    qx, qy, qz, qw = rotation.as_quat()
    
    # Define target position with orientation
    target_position = [0.553, -0.417, 0.234, 0.804, -0.525, 0.227, 0.161]  # [X, Y, Z, qw, qx, qy, qz]
    
    # Define target velocity (all zeros if not moving)
    target_velocity = [0.0] * 7
    
    # Define target acceleration (all zeros if not used)
    target_acceleration = [0.0] * 7  # Not used currently
    
    # Define proportional gain (ensure it's within [0.2, 100])
    proportional_gain = 25.0  # Example value
    
    try:
        # Execute servo control
        success = servo_cartesian(
            servo_target_position=target_position,
            servo_target_velocity=target_velocity,
            servo_target_acceleration=target_acceleration,
            gain=proportional_gain
        )
        
        if success:
            print("Servo control completed successfully.")
        else:
            print("Servo control failed or was interrupted.")
    except Exception as e:
        print(f"An error occurred: {e}")
