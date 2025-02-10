
# import CommunicationLibrary # importing communication library
# import time # importing time
# import json # importing json
# import logging # importing logging
# from neurapy.robot import Robot # importing robot module
# import time # importing time module
# from ruckig import InputParameter, OutputParameter, Result, Ruckig # importing ruckig module
 
# r = Robot() #settig r as the variable for the Robot
# r.gripper("on") # setting gripper on

# CONTROLLER_IP = "192.168.1.5" # IP address of the controller
# PORT = 11003 # #port number


# def test_ls(): # main function for calling every function.
#     robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
#     robot.connect_to_server(CONTROLLER_IP,PORT)  # communication between VC and robot is created

#     robot.pho_request_start_solution(252) # starting the solution
#     robot.pho_request_ls_scan(1) # ls scan
#     robot.pho_ls_wait_for_scan() # waiting for scan
#     robot.pho_request_get_objects(1, 5) # get objects
#     time.sleep(0.01)
#     robot.pho_get_current_position() # get current position
#     time.sleep(0.01)
#     robot.pho_request_ls_get_vision_system_status(1) # get vision system status
#     time.sleep(0.01)
#     robot.pho_request_change_solution(253) # change solution
#     time.sleep(0.01)
#     robot.pho_request_ls_scan(1) # ls scan
#     robot.pho_ls_wait_for_scan() # waiting for scan
#     robot.pho_request_get_objects(1, 5) # get objects
#     time.sleep(0.01) # sleep
#     robot.pho_request_get_running_solution() # get running solution
#     time.sleep(0.01)
#     #robot.pho_request_move_to_position()
#     # time.sleep(0.2)
#     # robot.pho_request_stop_solution()
#     # time.sleep(2)
#     robot.pho_request_get_available_solution() # get available solution
#     robot.close_connection()  #communication needs to be closed
#     time.sleep(0.01)

# def extract_object_coordinates(robot): # extract object coordinates [X,y,Z]
#     try:
#         # Replace 'objects' and 'coordinates' with actual attribute names from your response
#         objects = robot.response_data.objects  # Example attribute; adjust accordingly
        
#         if not objects:
#             logging.info("No objects are found")
#             return None

#         # For simplicity, consider the first detected object
#         first_object = objects[0]
#         object_coords = first_object.coordinates  # Example attribute; adjust accordingly

#         logging.info(f"Extracted Object Coordinates: {object_coords}")
#         return object_coords

#     except AttributeError:
#         logging.error("Failed to extract object coordinates. Check the response data structure.")
#         return None
#     except Exception as e:
#         logging.error(f"An error occurred while extracting object coordinates: {e}")
#         return None

# def format_coordinates(coords_mm): # function for formatting coordinates

#     try:
#         coords_m = [x / 1000.0 for x in coords_mm]
#         return coords_m
#     except TypeError:
#         logging.error("Invalid type for coordinates. Expected list or tuple.")
#         return None
#     except Exception as e:
#         logging.error(f"An error occurred while formatting coordinates: {e}")
#         return None

# def send_coordinates_to_robot(robot, coords): # function for sending coordinates to the robot
    
#     try:
#         # Replace 'pho_request_move_to_position' with the actual method name
#         # and adjust parameters as required by your CommunicationLibrary
#         robot.pho_request_move_to_position(coords[0], coords[1], coords[2])
#         logging.info(f"Sent move command to position: {coords}")
#     except AttributeError:
#         logging.error("The method 'pho_request_move_to_position' does not exist in CommunicationLibrary.")
#     except Exception as e:
#         logging.error(f"An error occurred while sending move command: {e}")




# def servo_j(message): # defining function for servoJ
#     #Switch to external servo mode

#     x = message[0] # extracting the x,y,z,a,b,c,d
#     y = message[1] # extracting the x,y,z,a,b,c,d
#     z = message[2] # extracting the x,y,z,a,b,c,d
#     a = message[3] # extracting the x,y,z,a,b,c,d
#     b = message[4] # extracting the x,y,z,a,b,c,d
#     c = message[5] # extracting the x,y,z,a,b,c,d
#     d = message[6] # extracting the x,y,z,a,b,c,d

#     r.activate_servo_interface('position') # activating the servo interface
#     dof = 6 # setting the DOF as 6 
#     otg = Ruckig(dof, 0.001)  # DoFs, control cycle

#     inp = InputParameter(dof) # setting the input parameter
#     out = OutputParameter(dof) # setting the output parameter
 
#     inp.current_position = r.get_current_joint_angles() # getting the current joint angles
#     inp.current_velocity = [0.]*dof
#     inp.current_acceleration = [0.]*dof
 
#     inp.target_position = [1.1625650370244778, -0.5774947959093657, -1.6300017754314295, 1.9807964651163987, 1.5676122261006906, 0.636066807616557] # target positon
#     inp.target_position = [x,y,z,a,b,c,d]
#     inp.max_velocity = [0.5]*dof # setting up the maximum velocity 
#     inp.max_acceleration = [3]*dof # setting up the maximum acceleration

   
#     inp = InputParameter(dof) # setting the input parameter
#     out = OutputParameter(dof) # setting the ouput parameters 
#     inp.current_position = r.get_current_joint_angles() # getting the current joint angles
#     inp.current_velocity = [0.]*dof # setting the current velocity as zero
#     inp.current_acceleration = [0.]*dof # setting the current acceleration as zero
 
#     inp.target_position = [1.1625650370244778, -0.5774947959093657, -1.6300017754314295, 1.9807964651163987, 1.5676122261006906, 0.636066807616557] # providing the target position
#     inp.target_acceleration = [0.]*dof # setting the target acceleration as zero.
#     r.gripper("on") # setting the gripper in On position.
 
#     inp.max_velocity = [0.5]*dof # defining the maximum velocity
#     inp.max_acceleration = [3]*dof # defining the maximum acceleration

#     inp.max_jerk = [10.]*dof
#     res = Result.Working
 
#     while res == Result.Working:
#         error_code = 0
#         res = otg.update(inp, out)

#         position = out.new_position # setting the new position 
#         velocity = out.new_velocity # setting the new velocity
#         acceleration = out.new_acceleration # setting the new acceleration 
 
#         error_code = r.servo_j(position, velocity, acceleration) # passing the error code variable with having servo_j function having position, velocity and acceleration.
#         print(error_code) # checking if the error is there or not 
#         scaling_factor = r.get_servo_trajectory_scaling_factor() # getting the servo trajectory scaling factors.
#         out.pass_to_input(inp)
#         time.sleep(0.001) # setting the time sleep to 0.001 seconds

#     r.deactivate_servo_interface() # deactivating the servo interface
 
<<<<<<< HEAD
#     r.stop() # stopped the robot

# r.gripper("off") # setting gripper off


# def move_robot_to_position(robot, target_coords, tolerance=0.01, timeout=30): # function for moving robot to position



#     try:
#         start_time = time.time() # setting the start time
#         while True:
#             # Replace 'get_current_position' with the actual method to retrieve the robot's current position
#             current_coords = robot.pho_get_current_position() # getting the current position
#             distance = ((current_coords[0] - target_coords[0]) ** 2 +
#                         (current_coords[1] - target_coords[1]) ** 2 +
#                         (current_coords[2] - target_coords[2]) ** 2) ** 0.5
#             if distance <= tolerance: # setting the tolerance
#                 logging.info(f"Robot reached target position: {current_coords}")
#                 break
#             if time.time() - start_time > timeout: # setting the timeout
#                 raise TimeoutError("Robot did not reach the target position in time.")
#             time.sleep(0.5)
#     except AttributeError: # error handling
#         logging.error("The method 'get_current_position' does not exist in CommunicationLibrary.")
#     except Exception as e: # error handling
#         logging.error(f"An error occurred while moving the robot: {e}")

# def test_ls(): # main function for calling every function.
#     """
#     Tests the LS (Laser Scan) functionality of the robot.
#     Extracts object coordinates from the camera and sends them to the robotic controller to move the robot.
#     """
#     robot = CommunicationLibrary.RobotRequestResponseCommunication()  # Create robot communication object
#     try:
#         logging.info(f"Connecting to robot at {CONTROLLER_IP}:{PORT}")
#         robot.connect_to_server(CONTROLLER_IP, PORT)  # Establish communication

#         logging.info("Starting solution 252")
#         robot.pho_request_start_solution(252) # Start solution 252

#         logging.info("Initiating LS scan")
#         robot.pho_request_ls_scan(1)  # Start LS scan with parameter 1
#         robot.pho_ls_wait_for_scan()    # Wait for the scan to complete

#         logging.info("Requesting objects detected in the scan")
#         robot.pho_request_get_objects(1, 5)  # Get objects detected (parameters may vary)
#         time.sleep(0.1)  # Short delay to ensure response is received

#         logging.info("Retrieving vision system status")
#         robot.pho_request_ls_get_vision_system_status(1) # Get vision system status
#         time.sleep(0.1)

#         logging.info("Changing solution to 253")
#         robot.pho_request_change_solution(253) # Change solution
#         time.sleep(0.1)

#         logging.info("Initiating another LS scan")
#         robot.pho_request_ls_scan(1) # Start another LS scan
#         robot.pho_ls_wait_for_scan() # Wait for the scan to complete

#         logging.info("Requesting objects detected in the second scan")
#         robot.pho_request_get_objects(1, 5) # Get objects detected in the second scan
#         time.sleep(0.1)

#         logging.info("Retrieving running solution")
#         robot.pho_request_get_running_solution() # Get running solution
#         time.sleep(0.1)

#         logging.info("Retrieving available solutions")
#         robot.pho_request_get_available_solution() # Get available solutions

#         # Extract object coordinates from the response
#         object_coords = extract_object_coordinates(robot) # Extract object coordinates
#         if object_coords: # if object coordinates are found
#             # Format coordinates (e.g., convert from mm to m)
#             formatted_coords = format_coordinates(object_coords)
#             if formatted_coords: # if formatting is successful
#                 logging.info(f"Formatted Object Coordinates (meters): {formatted_coords}")

#                 # Send coordinates to the robot to move
#                 send_coordinates_to_robot(robot, formatted_coords)

#                 # Optional: Command the robot to move to the coordinates and wait until it reaches
#                 # Uncomment the line below if you wish to perform this action
#                 # move_robot_to_position(robot, formatted_coords)
#             else:
#                 logging.warning("Failed to format object coordinates.")
#         else:
#             logging.warning("No object coordinates found in the response.")

#     except CommunicationLibrary.ConnectionError as ce:
#         logging.error(f"Connection failed: {ce}")
#     except CommunicationLibrary.CommandError as cmd_err:
#         logging.error(f"Command failed: {cmd_err}")
#     except TimeoutError as te:
#         logging.error(f"Operation timed out: {te}")
#     except Exception as e:
#         logging.error(f"An unexpected error occurred: {e}")
#     finally:
#         logging.info("Closing connection to the robot.")
#         robot.close_connection()
#         time.sleep(0.1)  # Short delay after closing the connection

# def calibration_extrinsic(): # function for extrinsic calibration
#     robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
#     robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

#     robot.pho_request_start_automatic_calibration(6,1) # start automatic calibration
#     # Load the JSON data
#     file_path = 'extrinsic_calib_points.json'
#     json_data = load_json_file(file_path)

#     # add 9 calibration point
#     for point in json_data: # for each point
#         translation_mm = point["translation"] # translation
#         quaternion = point["quaternion"] # quaternion
#         translation_m = [x * 1000 for x in translation_mm] # mm to m
#         tool_pose = translation_m + quaternion

#         robot.pho_request_add_calibration_point(tool_pose) # add calibration point
#         time.sleep(2)

#     robot.pho_request_save_automatic_calibration() # save automatic calibration
#     time.sleep(2)
#     robot.pho_request_stop_automatic_calibration() # stop automatic calibration


# def calibration_handeye(): # function for handeye calibration
#     robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
#     robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

#     robot.pho_request_start_automatic_calibration(6, 2) # start automatic calibration
#     # Load the JSON data
#     file_path = 'handeye_calib_points.json'
#     json_data = load_json_file(file_path)

#     # add 9 calibration point
#     for point in json_data: # for each point
#         translation_mm = point["translation"] # translation
#         quaternion = point["quaternion"] # quaternion
#         translation_m = [x * 1000 for x in translation_mm]  # mm to m
#         tool_pose = translation_m + quaternion # tool pose

#         robot.pho_request_add_calibration_point(tool_pose) # add calibration point
#         time.sleep(2) # sleep for 2 seconds


#     #robot.pho_request_save_automatic_calibration()

#     robot.pho_request_save_automatic_calibration() # save automatic calibration
#     time.sleep(2)
#     robot.pho_request_stop_automatic_calibration() # stop automatic calibration


# # Function to load JSON data from a file
# def load_json_file(file_path): # function to load JSON data from a file
#     with open(file_path, 'r') as file: # open file
#         data = json.load(file) # load data
#     return data # return data


# if __name__ == '__main__': # main function
#     # calibration_handeye()
#     calibration_extrinsic() # extrinsic calibration
#     test_ls() # calling the test_ls function
#     #test_bps()

#     while True: # main loop
#         test_ls() # calling the test_ls function
#         #test_bps()

#!/usr/bin/env python3
import socket
import time
import CommunicationLibrary
import random
import math
import numpy as np

SOCKET_RECV_TIMEOUT = 5
ROBOT_CONTROLLER_IP = "192.168.1.2"
PORT = 11004

# Requests
JOINT_STATE_TYPE = 1
TOOL_POSE_TYPE = 2

# variables for get_joint_state() + get_tool_pose()
init_joint_state = [0, 0, 0, 0, 0, 0]
base_quat = np.array([1, 0, 0, 0])


def get_joint_state(init_joint_state):
    if not hasattr(get_joint_state, 'joint_state'):
        get_joint_state.joint_state = init_joint_state # create function attribute for storing actual joint_state
        print(f"Initialized: {get_joint_state.joint_state}")
        get_joint_state.counter = 0 # create counter for switching

    # increments of joints
    divider = 100  # affect size of increment - inverse proportion
    inc_j1 = (random.random() * math.pi / divider)
    inc_j2 = (random.random() * math.pi / divider)
    inc_j3 = (random.random() * math.pi / divider)
    inc_j4 = (random.random() * math.pi / divider)
    inc_j5 = (random.random() * math.pi / divider)
    inc_j6 = (random.random() * math.pi / divider)

    # increment +/- number
    pos_neg = [1, 1, 1, 1, 1, 1] # affect the sign of increment
    # set different combinations for changing robot motions
    if get_joint_state.counter < 30:
        pos_neg = [-1, 1, 1, 1, 1, -1]
    elif get_joint_state.counter < 60:
        pos_neg = [1, 0, -1, -1, -1, 1]
    elif get_joint_state.counter < 90:
        pos_neg = [1, -1, 1, 1, -1, 1]
    elif get_joint_state.counter < 120:
        pos_neg = [1, 0, -1, -1, 1, -1]
        get_joint_state.counter = 0

    # incrementing joint_state
    get_joint_state.joint_state = [
        get_joint_state.joint_state[0] + inc_j1 * pos_neg[0],
        get_joint_state.joint_state[1] + inc_j2 * pos_neg[1],
        get_joint_state.joint_state[2] + inc_j3 * pos_neg[2],
        get_joint_state.joint_state[3] + inc_j4 * pos_neg[3],
        get_joint_state.joint_state[4] + inc_j5 * pos_neg[4],
        get_joint_state.joint_state[5] + inc_j6 * pos_neg[5]]

    # set joint limits
    upper_limit = [3.14, 0.6, 1.13, 3.14, 0.6, 3.14]
    lower_limit = [0, -0.6, -3, 0, -0.6, -3.14]
    count = 0
    # joint limits check
    for joint_value in get_joint_state.joint_state:
        if joint_value > upper_limit[count]:
            get_joint_state.joint_state[count] = upper_limit[count]
        elif joint_value < lower_limit[count]:
            get_joint_state.joint_state[count] = lower_limit[count]
        count += 1

    # increment counter
    get_joint_state.counter += 1
    return get_joint_state.joint_state


def get_tool_pose(init_quaternion):
    if not hasattr(get_tool_pose, 'actual_quaternion'):
        get_tool_pose.actual_quaternion = init_quaternion # create function attribute to store actual quaternion value
        print(f"Initialized: {get_tool_pose.actual_quaternion}")
        # create counters for functions
        get_tool_pose.counter = 0  # changing rotation
        get_tool_pose.counter_circle = 0  # circular motion

    # define circular motion of tool_pose - around z-axis
    num_frames = 100
    radius = 1000
    alfa = 2 * np.pi * get_tool_pose.counter_circle / num_frames
    x = radius * np.cos(alfa)
    y = radius * np.sin(alfa)
    z = 500

    # Define the rotation of quaternion
    theta = np.pi / 50  # radians
    rot = np.sin(theta / 2)
    rotation_quat = np.array([np.cos(theta / 2), 0, 0, 0]) # no rotation

    if get_tool_pose.counter < num_frames / 4:
        rotation_quat = np.array([np.cos(theta / 2), 0, 0, rot])  # z-axis
    elif get_tool_pose.counter < 2 * num_frames / 4:
        rotation_quat = np.array([np.cos(theta / 2), 0, rot, 0])  # y-axis
    elif get_tool_pose.counter < 3 * num_frames / 4:
        rotation_quat = np.array([np.cos(theta / 2), rot, 0, 0])  # x-axis
    elif get_tool_pose.counter < num_frames:
        rotation_quat = np.array([np.cos(theta / 2), 0, 0, 0])  # no rotation
        get_tool_pose.counter = 0  # reset counter


    rotation_quat = normalize_quaternion(rotation_quat) # Normalize the rotation quaternion (just in case)

    result_quat = quaternion_multiply(get_tool_pose.actual_quaternion, rotation_quat) # Apply the rotation to the actual quaternion

    get_tool_pose.actual_quaternion = normalize_quaternion(result_quat) # Normalize the resulting quaternion (just in case)

    # increment counters
    get_tool_pose.counter += 1
    get_tool_pose.counter_circle += 1

    return [x, y, z, result_quat[0], result_quat[1], result_quat[2], result_quat[3]]


# Function to multiply two quaternions
def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])


# Function to normalize a quaternion
def normalize_quaternion(q):
    norm = np.linalg.norm(q)
    return q / norm


def test_loop_communication():
    server = CommunicationLibrary.RobotStateCommunication() # create server object
    server.create_server(ROBOT_CONTROLLER_IP, PORT)
    server.wait_for_client()
    while True:
        try:
            server.send_joint_state()
            server.send_tool_pose()
        except socket.error:
            print('Communication lost. Trying to reconnect...')
            break

    r.stop() # stopped the robot

r.gripper("off") # setting gripper off


def move_robot_to_position(robot, target_coords, tolerance=0.01, timeout=30): # function for moving robot to position

    try:
        start_time = time.time() # setting the start time
        while True:
            # Replace 'get_current_position' with the actual method to retrieve the robot's current position
            current_coords = robot.pho_get_current_position() # getting the current position
            distance = ((current_coords[0] - target_coords[0]) ** 2 +
                        (current_coords[1] - target_coords[1]) ** 2 +
                        (current_coords[2] - target_coords[2]) ** 2) ** 0.5
            if distance <= tolerance: # setting the tolerance
                logging.info(f"Robot reached target position: {current_coords}")
                break
            if time.time() - start_time > timeout: # setting the timeout
                raise TimeoutError("Robot did not reach the target position in time.")
            time.sleep(0.5)
    except AttributeError: # error handling
        logging.error("The method 'get_current_position' does not exist in CommunicationLibrary.")
    except Exception as e: # error handling
        logging.error(f"An error occurred while moving the robot: {e}")

def test_ls(): # main function for calling every function.
    """
    Tests the LS (Laser Scan) functionality of the robot.
    Extracts object coordinates from the camera and sends them to the robotic controller to move the robot.
    """
    robot = CommunicationLibrary.RobotRequestResponseCommunication()  # Create robot communication object
    try:
        logging.info(f"Connecting to robot at {CONTROLLER_IP}:{PORT}")
        robot.connect_to_server(CONTROLLER_IP, PORT)  # Establish communication

        logging.info("Starting solution 252")
        robot.pho_request_start_solution(252) # Start solution 252

        logging.info("Initiating LS scan")
        robot.pho_request_ls_scan(1)  # Start LS scan with parameter 1
        robot.pho_ls_wait_for_scan()    # Wait for the scan to complete

        logging.info("Requesting objects detected in the scan")
        robot.pho_request_get_objects(1, 5)  # Get objects detected (parameters may vary)
        time.sleep(0.1)  # Short delay to ensure response is received

        logging.info("Retrieving vision system status")
        robot.pho_request_ls_get_vision_system_status(1) # Get vision system status
        time.sleep(0.1)

        logging.info("Changing solution to 253")
        robot.pho_request_change_solution(253) # Change solution
        time.sleep(0.1)

        logging.info("Initiating another LS scan")
        robot.pho_request_ls_scan(1) # Start another LS scan
        robot.pho_ls_wait_for_scan() # Wait for the scan to complete

        logging.info("Requesting objects detected in the second scan")
        robot.pho_request_get_objects(1, 5) # Get objects detected in the second scan
        time.sleep(0.1)


        time.sleep(0.1)


if __name__ == "__main__":
    test_loop_communication()
