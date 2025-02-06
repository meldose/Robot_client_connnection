#!/usr/bin/env python3

import CommunicationLibrary # importing communication library
import time # importing time
import json # importing json
import logging #    importing logging

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


# Enhanced Robot Movement Code with Improvements
def move_robot_to_position(robot, target_coords, tolerance=0.01, timeout=30, check_interval=0.05):
    """
    Moves the robot to the target coordinates with improved error handling, dynamic adjustments, and safety measures.

    :param robot: Robot object with servo_j, pho_get_current_position, and optional stop_motion methods
    :param target_coords: Tuple of (x, y, z) target coordinates
    :param tolerance: Position tolerance to consider as "reached"
    :param timeout: Maximum allowed time to reach the target (in seconds)
    :param check_interval: Interval to check the robot's position (in seconds)
    """
    def stop_robot():
        try:
            robot.stop_motion()  # Hypothetical stop function
            logging.info("Emergency stop triggered.")
        except AttributeError:
            logging.warning("Stop function not available in the robot API.")

    try:
        # Initiate movement
        robot.servo_j(target_coords)
        logging.info(f"Movement started towards: {target_coords}")

        start_time = time.time()
        previous_distance = float('inf')  # Initialize with a large number

        while True:
            # Check for timeout first
            if time.time() - start_time > timeout:
                raise TimeoutError("Robot did not reach the target position in time.")

            # Retrieve current position
            current_coords = robot.pho_get_current_position()

            # Calculate distance to target
            distance = ((current_coords[0] - target_coords[0]) ** 2 +
                        (current_coords[1] - target_coords[1]) ** 2 +
                        (current_coords[2] - target_coords[2]) ** 2) ** 0.5

            # Log progress
            logging.info(f"Current position: {current_coords}, Distance to target: {distance:.4f}")

            # Check if within tolerance
            if distance <= tolerance:
                logging.info(f"Robot reached target position: {current_coords}")
                break

            # Adjust movement dynamically only if progress stalls
            if distance >= previous_distance - 0.001:  # No significant improvement
                logging.info("Re-adjusting movement to correct trajectory.")
                robot.servo_j(target_coords)

            previous_distance = distance

            time.sleep(check_interval)

    except AttributeError as e:
        logging.error(f"Missing method in robot API: {e}")
        stop_robot()
    except TimeoutError as e:
        logging.warning(e)
        stop_robot()
    except Exception as e:
        logging.error(f"An unexpected error occurred: {e}")
        stop_robot()


def calibration_extrinsic():
    robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
    robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

    robot.pho_request_start_automatic_calibration(6,1)
    # Load the JSON data
    file_path = 'extrinsic_calib_points.json'
    json_data = load_json_file(file_path)

    # add 9 calibration point
    for point in json_data:
        translation_mm = point["translation"]
        quaternion = point["quaternion"]
        translation_m = [x * 1000 for x in translation_mm] # mm to m
        tool_pose = translation_m + quaternion

        robot.pho_request_add_calibration_point(tool_pose)
        time.sleep(2)

    robot.pho_request_save_automatic_calibration()
    time.sleep(2)
    robot.pho_request_stop_automatic_calibration()


def calibration_handeye():
    robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
    robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

    robot.pho_request_start_automatic_calibration(6, 2)
    # Load the JSON data
    file_path = 'handeye_calib_points.json'
    json_data = load_json_file(file_path)

    # add 9 calibration point
    for point in json_data:
        translation_mm = point["translation"]
        quaternion = point["quaternion"]
        translation_m = [x * 1000 for x in translation_mm]  # mm to m
        tool_pose = translation_m + quaternion

        robot.pho_request_add_calibration_point(tool_pose)
        time.sleep(2)


    #robot.pho_request_save_automatic_calibration()

    robot.pho_request_save_automatic_calibration()
    time.sleep(2)
    robot.pho_request_stop_automatic_calibration()


# Function to load JSON data from a file
def load_json_file(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data


if __name__ == '__main__':
    # calibration_handeye()
    calibration_extrinsic()
    test_ls()
    #test_bps()
 
    while True:
        test_ls()
        #test_bps()

