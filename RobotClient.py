#!/usr/bin/env python3

import CommunicationLibrary # importing communication library
import time # importing time
import json # importing json
import logging #    importing logging
from neurapy.robot import Robot
r=Robot()

CONTROLLER_IP = "192.168.1.5" # IP address of the controller
PORT = 11003 # port number


def test_ls(): # test ls
    robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
    robot.connect_to_server(CONTROLLER_IP,PORT)  # communication between VC and robot is created

    robot.pho_request_start_solution(252) # start solution
    robot.pho_request_ls_scan(1) # ls scan
    robot.pho_ls_wait_for_scan() # wait for scan
    robot.pho_request_get_objects(1, 5) # get objects
    time.sleep(0.1) # sleep for 0.1 second
    robot.pho_get_current_position() #  get current position
    time.sleep(0.1) # sleep for 0.1 second
    robot.pho_request_ls_get_vision_system_status(1) # get vision system status
    time.sleep(0.1) # sleep for 0.1 second
    robot.pho_request_change_solution(253) # change solution
    time.sleep(0.1) # sleep for 0.1 second
    robot.pho_request_ls_scan(1) # ls scan
    robot.pho_ls_wait_for_scan() # wait for scan
    robot.pho_request_get_objects(1, 5) # get objects
    time.sleep(0.1) # sleep for 0.1 second
    robot.pho_request_get_running_solution() # get running solution
    time.sleep(0.1) # sleep for 0.1 second
    #robot.pho_request_move_to_position()
    # time.sleep(0.2)
    # robot.pho_request_stop_solution()
    # time.sleep(2)
    robot.pho_request_get_available_solution()
    robot.close_connection()  #communication needs to be closed
    time.sleep(0.1)

def extract_object_coordinates(robot): # extract object coordinates [X,y,Z]
    try:
        # Replace 'objects' and 'coordinates' with actual attribute names from your response
        objects = robot.response_data.objects  # Example attribute; adjust accordingly
        
        if not objects:
            logging.info("No objects are found")
            return None

        # For simplicity, consider the first detected object
        first_object = objects[0] # Example attribute; adjust accordingly
        object_coords = first_object.coordinates  # Example attribute; adjust accordingly

        logging.info(f"Extracted Object Coordinates: {object_coords}")
        return object_coords

    except AttributeError: # error handling
        logging.error("Failed to extract object coordinates. Check the response data structure.") # error message
        return None # return None
    except Exception as e:
        logging.error(f"An error occurred while extracting object coordinates: {e}")
        return None

def format_coordinates(coords_mm): # function for formatting coordinates

    try:
        coords_m = [x / 1000.0 for x in coords_mm]
        return coords_m
    except TypeError:
        logging.error("Invalid type for coordinates. Expected list or tuple.")
        return None
    except Exception as e:
        logging.error(f"An error occurred while formatting coordinates: {e}")
        return None

def send_coordinates_to_robot(robot, coords): # function for sending coordinates to the robot
    
    try:
        # Replace 'pho_request_move_to_position' with the actual method name
        # and adjust parameters as required by your CommunicationLibrary
        robot.pho_request_move_to_position(coords[0], coords[1], coords[2])
        logging.info(f"Sent move command to position: {coords}")
    except AttributeError:
        logging.error("The method 'pho_request_move_to_position' does not exist in CommunicationLibrary.")
    except Exception as e:
        logging.error(f"An error occurred while sending move command: {e}")

# def move_robot_to_position(robot, target_coords, tolerance=0.01, timeout=30): # function for moving robot to position

#     try:
#         start_time = time.time()
#         while True:
#             # Replace 'get_current_position' with the actual method to retrieve the robot's current position
#             current_coords = robot.pho_get_current_position()
#             distance = ((current_coords[0] - target_coords[0]) ** 2 +
#                         (current_coords[1] - target_coords[1]) ** 2 +
#                         (current_coords[2] - target_coords[2]) ** 2) ** 0.5
#             if distance <= tolerance:
#                 logging.info(f"Robot reached target position: {current_coords}")
#                 break
#             if time.time() - start_time > timeout:
#                 raise TimeoutError("Robot did not reach the target position in time.")
#             time.sleep(0.5)
#     except AttributeError:
#         logging.error("The method 'get_current_position' does not exist in CommunicationLibrary.")
#     except Exception as e:
#         logging.error(f"An error occurred while moving the robot: {e}")



##### first trail ################
def move_robot_to_position(robot, target_coords, tolerance=0.01, timeout=30): # function for moving robot to position
    try:
        robot.activate_servo_interface('position') # activate servo interface
        dof = 6 # degrees of freedom
        otg = Ruckig(dof, 0.001) # create Ruckig object

        inp = InputParameter(dof) # create InputParameter object
        out = OutputParameter(dof) # create OutputParameter object

        inp.current_position = r.get_current_joint_angles() # get current joint angles
        inp.current_velocity = [0.0] * dof # set current velocity to zero
        inp.current_acceleration = [0.0] * dof # set current acceleration to zero

        inp.target_position = target_coords  # Target joint positions
        inp.max_velocity = [0.5] * dof # set maximum velocity
        inp.max_acceleration = [3.0] * dof # set maximum acceleration
        inp.max_jerk = [10.0] * dof #   set maximum jerk

        res = Result.Working # set result to working
        start_time = time.time() # set start time

        while res == Result.Working: # while result is working
            if time.time() - start_time > timeout: # if time out
                raise TimeoutError("Robot did not reach the target position in time.")

            res = otg.update(inp, out) # update

            position = out.new_position # get new position
            velocity = out.new_velocity # get new velocity
            acceleration = out.new_acceleration # get new acceleration

            error_code = r.servo_j(position, velocity, acceleration) # send to robot
            logging.info(f"Error Code: {error_code}") # log

            scaling_factor = r.get_servo_trajectory_scaling_factor() # get scaling factor
            out.pass_to_input(inp) # pass to input

            current_coords = r.get_current_joint_angles() # get current joint angles
            distance = sum((c - t) ** 2 for c, t in zip(current_coords, target_coords)) ** 0.5 # calculate distance
            if distance <= tolerance: # if distance is less than tolerance
                logging.info(f"Robot reached target position: {current_coords}") # log
                break

            time.sleep(0.001) # sleep

        robot.deactivate_servo_interface() # deactivate servo interface
        robot.stop() # stop

    except AttributeError as e: # error handling
        logging.error(f"Attribute error: {e}") # error message
    except Exception as e: # error handling
        logging.error(f"An error occurred while moving the robot: {e}") # error message

r.gripper("off") # set gripper off


def calibration_extrinsic(): # function for extrinsic calibration
    robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
    robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

    robot.pho_request_start_automatic_calibration(6,1) # start automatic calibration
    # Load the JSON data
    file_path = 'extrinsic_calib_points.json'
    json_data = load_json_file(file_path)

    # add 9 calibration point
    for point in json_data: # for each point
        translation_mm = point["translation"]
        quaternion = point["quaternion"]
        translation_m = [x * 1000 for x in translation_mm] # mm to m
        tool_pose = translation_m + quaternion

        robot.pho_request_add_calibration_point(tool_pose) # add point
        time.sleep(2) # sleep for 2 seconds

    robot.pho_request_save_automatic_calibration() # save calibration
    time.sleep(2) # sleep for 2 seconds
    robot.pho_request_stop_automatic_calibration() # stop calibration


def calibration_handeye(): # function for handeye calibration
    robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
    robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

    robot.pho_request_start_automatic_calibration(6, 2) # start automatic calibration
    # Load the JSON data 
    file_path = 'handeye_calib_points.json' # file path
    json_data = load_json_file(file_path) # load data

    # add 9 calibration point
    for point in json_data: # for each point
        translation_mm = point["translation"] # translation
        quaternion = point["quaternion"] # quaternion
        translation_m = [x * 1000 for x in translation_mm]  # mm to m
        tool_pose = translation_m + quaternion # tool pose

        robot.pho_request_add_calibration_point(tool_pose) # add point
        time.sleep(2) # sleep for 2 seconds


    #robot.pho_request_save_automatic_calibration()

    robot.pho_request_save_automatic_calibration() # save calibration
    time.sleep(2) # sleep for 2 seconds
    robot.pho_request_stop_automatic_calibration() # stop calibration


# Function to load JSON data from a file
def load_json_file(file_path): # function to load json
    with open(file_path, 'r') as file: # open file
        data = json.load(file) # load data
    return data # return data


if __name__ == '__main__': # main function
    # calibration_handeye() 
    calibration_extrinsic() # extrinsic calibration
    test_ls() # 
    #test_bps()
 
    while True: # infinite loop
        test_ls() # test ls
        #test_bps()

