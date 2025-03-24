
# -------------------------------------------------------------------
#                      IMPORTS
# -------------------------------------------------------------------
import Communication_Library_VS_2ID # importing communication library
import time # importing time
import json # importing json
import logging # importing logging
from neurapy.robot import Robot # importing robot module
import time # importing time module
from ruckig import InputParameter, OutputParameter, Result, Ruckig # importing ruckig module
from Communication_Library_VS_2ID import ServoX,RobotRequestResponseCommunication,RobotStateCommunication # for running robot and camera
# from CommunicationLibrary import RobotRequestResponseCommunication,RobotStateCommunication # for running camera only
r = Robot() #settig r as the variable for the Robot
r.gripper("on") # setting gripper on

CONTROLLER_IP = "192.168.1.5" # IP address of the controller
PORT = 11003 # #port number

# -------------------------------------------------------------------
#                      MAIN FUNCTION
# -------------------------------------------------------------------
def test_ls(): # main function for calling every function.    
    robot = Communication_Library_VS_2ID.RobotRequestResponseCommunication()  # object is created
    robot.connect_to_server(CONTROLLER_IP,PORT)  # communication between VC and robot is created

    robot.pho_request_ls_scan(vs_id_1=1) # ls scan for object 1 (trapezoid)    
    time.sleep(0.01) # setting time sleep
    robot.pho_ls_wait_for_scan(vs_id_1=1) # waiting for scan for object 1 (trapezoid)
    time.sleep(0.01) # setting time sleep
    robot.pho_request_get_objects(vs_id_1=1,number_of_objects_1=1) # get objects for first object trapezoid and vision system 1
    time.sleep(0.01) # setting time sleep
    robot.pho_request_start_solution(252) # starting the solution
    time.sleep(0.01)
    robot.pho_request_ls_get_vision_system_status(vs_id_1=1) # get vision system status for first
    time.sleep(0.01) # setting time sleep
    robot.pho_request_ls_scan_2(vs_id_2=2) # ls scan for vision system 2 (pipe)
    time.sleep(0.01) # setting time sleep
    robot.pho_ls_wait_for_scan_2(vs_id_2=2) # waiting for scan for vision system2 
    time.sleep(0.01) # setting time sleep
    robot.pho_request_ls_get_vision_system_status_2(vs_id_2=2) # get vision system status for second
    time.sleep(0.01) # setting time sleep
    robot.pho_request_change_solution(253) # change solution
    time.sleep(0.01) # setting time sleep
    # robot.pho_request_ls_scan(vs_id_1=1) # ls scan for object 1 (trapezoid)
    # time.sleep(0.01) # setting time sleep
    # robot.pho_ls_wait_for_scan(vs_id_1=1) # waiting for scan for object 1 (trapezoid)
    # time.sleep(0.01) # setting time sleep
    # robot.pho_request_get_objects(vs_id_1=1,number_of_objects_1=1) # get objects for first object trapezoid and vision system 1
    # time.sleep(0.01) # setting time sleep
    # robot.pho_request_ls_scan_2(vs_id_2=2) # ls scan for vision system 2 (pipe)
    # time.sleep(0.01) # setting time sleep
    # robot.pho_ls_wait_for_scan_2(vs_id_2=2) # waiting for scan for vision system2 
    # time.sleep(0.01) # setting time sleep
    # robot.pho_request_get_objects_2(vs_id_2=2,number_of_objects_2=2) # get objects for object2 and vision system 2(pipe)
    # time.sleep(0.01) # setting time sleep
    # robot.pho_request_get_running_solution() # get running solution
    # time.sleep(0.01) # setting time sleep
    # robot.pho_request_get_available_solution() # get available solution
    # time.sleep(0.01) # setting time sleep
    # robot.close_connection()  #communication needs to be closed
    # time.sleep(0.01) # setting time sleep

def extract_object_coordinates(robot): # extract object coordinates [X,y,Z]
    try:
        # Replace 'objects' and 'coordinates' with actual attribute names from your response
        objects = robot.response_data.objects  # Example attribute; adjust accordingly
        
        if not objects: # if the objects not found
            logging.info("No objects are found") # logging error
            return None # return None

        # For simplicity, consider the first detected object
        first_object = objects[0] # setting the first_object having the first element of the response_data.objects
        object_coords = first_object.coordinates  # Example attribute; adjust accordingly

        logging.info(f"Extracted Object Coordinates: {object_coords}") # giving the logging error
        return object_coords # return the object_coords

    except AttributeError:
        logging.error("Failed to extract object coordinates. Check the response data structure.") # logging error
        return None # return None
    except Exception as e:
        logging.error(f"An error occurred while extracting object coordinates: {e}") # logging error
        return None # return None

def format_coordinates(coords_mm): # function for formatting coordinates

    try:
        coords_m = [x / 1000.0 for x in coords_mm] # setting the coords in mm
        return coords_m # return the coordinates in mm
    except TypeError:
        logging.error("Invalid type for coordinates. Expected list or tuple.") # logging error
        return None # return None
    except Exception as e:
        logging.error(f"An error occurred while formatting coordinates: {e}") # logging error 
        return None # return None

def send_coordinates_to_robot(robot, coords): # function for sending coordinates to the robot
    
    try:
        robot.pho_request_move_to_position(coords[0], coords[1], coords[2]) # requesting the robot to move postion with respective coords
        logging.info(f"Sent move command to position: {coords}") # logging info 
    except AttributeError:
        logging.error("The method 'pho_request_move_to_position' does not exist in CommunicationLibrary.") # loggig error
    except Exception as e:
        logging.error(f"An error occurred while sending move command: {e}") # logging error

def move_robot_to_position(robot, target_coords, tolerance=0.01, timeout=30): # deifining the function for robot to move the position

    robot=RobotRequestResponseCommunication() # object is created
    servo=ServoX.servo_x() # calling servo function
    r.gripper("off") # setting gripper off
    try:
        start_time = time.time() # setting the time
        while True:
            # Replace 'get_current_position' with the actual method to retrieve the robot's current position
            current_coords = robot.pho_get_current_position() # getting the current position
            distance = ((current_coords[0] - target_coords[0]) ** 2 +
                        (current_coords[1] - target_coords[1]) ** 2 +
                        (current_coords[2] - target_coords[2]) ** 2) ** 0.5
            if distance <= tolerance: # if the distance is less than the tolerance
                logging.info(f"Robot reached target position: {current_coords}") # loggging error
                break
            if time.time() - start_time > timeout: # chceking if the start time is greater than the timeout
                raise TimeoutError("Robot did not reach the target position in time.") # raise the TimeError
            time.sleep(0.5) # settig the time
    except AttributeError:
        logging.error("The method 'get_current_position' does not exist in CommunicationLibrary.") # logging error
    except Exception as e:
        logging.error(f"An error occurred while moving the robot: {e}") # logging error
    return servo # return sevo

# -------------------------------------------------------------------
#                      EXTRINSIC CALIBRATION
# -------------------------------------------------------------------
def calibration_extrinsic(): # function for extrinsic calibration
    robot = Communication_Library_VS_2ID.RobotRequestResponseCommunication()  # object is created
    robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

    robot.pho_request_start_automatic_calibration(6,1) # start automatic calibration
    # Load the JSON data
    file_path = 'extrinsic_calib_points.json'
    json_data = load_json_file(file_path)

    # add 9 calibration point
    for point in json_data: # for each point
        translation_mm = point["translation"] # translation
        quaternion = point["quaternion"] # quaternion
        translation_m = [x * 1000 for x in translation_mm] # mm to m
        tool_pose = translation_m + quaternion

        robot.pho_request_add_calibration_point(tool_pose) # add calibration point
        time.sleep(2)

    robot.pho_request_save_automatic_calibration() # save automatic calibration
    time.sleep(2)
    robot.pho_request_stop_automatic_calibration() # stop automatic calibration

# -------------------------------------------------------------------
#                      HAND-EYE CALIBRATION
# -------------------------------------------------------------------
def calibration_handeye(): # function for handeye calibration
    robot = Communication_Library_VS_2ID.RobotRequestResponseCommunication()  # object is created
    robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

    robot.pho_request_start_automatic_calibration(6, 2) # start automatic calibration
    # Load the JSON data
    file_path = 'handeye_calib_points.json'
    json_data = load_json_file(file_path)

    # add 9 calibration point
    for point in json_data: # for each point
        translation_mm = point["translation"] # translation
        quaternion = point["quaternion"] # quaternion
        translation_m = [x * 1000 for x in translation_mm]  # mm to m
        tool_pose = translation_m + quaternion # tool pose

        robot.pho_request_add_calibration_point(tool_pose) # add calibration point
        time.sleep(2) # sleep for 2 seconds

    robot.pho_request_save_automatic_calibration() # save automatic calibration
    time.sleep(2)
    robot.pho_request_stop_automatic_calibration() # stop automatic calibration


# Function to load JSON data from a file
def load_json_file(file_path): # function to load JSON data from a file
    with open(file_path, 'r') as file: # open file
        data = json.load(file) # load data
    return data # return data


if __name__ == '__main__': # main function
    calibration_extrinsic() # extrinsic calibration
    test_ls() # calling the test_ls function
    # ServoJ(robot=r).servo_j()
    #test_bps()

    while True: # main loop
        test_ls() # calling the test_ls function
        #test_bps()

