
import CommunicationLibrary # importing communication library
import time # importing time
import json # importing json
import logging # importing logging
from neurapy.robot import Robot # importing robot module
import time # importing time module
from ruckig import InputParameter, OutputParameter, Result, Ruckig # importing ruckig module
from CommunicationLibrary import ServoX,RobotRequestResponseCommunication,RobotStateCommunication
r = Robot() #settig r as the variable for the Robot
r.gripper("on") # setting gripper on

CONTROLLER_IP = "192.168.1.5" # IP address of the controller
PORT = 11003 # #port number


# -------------------------------------------------------------------
#                      MAIN FUNCTION
# -------------------------------------------------------------------
def test_ls(): # main function for calling every function.
    robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
    robot.connect_to_server(CONTROLLER_IP,PORT)  # communication between VC and robot is created

    robot.pho_request_start_solution(252) # starting the solution
    robot.pho_request_ls_scan(1) # ls scan
    robot.pho_ls_wait_for_scan() # waiting for scan
    robot.pho_request_get_objects(1, 5) # get objects
    time.sleep(0.01)
    # robot.pho_get_current_position() # get current position
    time.sleep(0.01)
    robot.pho_request_ls_get_vision_system_status(1) # get vision system status
    time.sleep(0.01)
    robot.pho_request_change_solution(253) # change solution
    time.sleep(0.01)
    robot.pho_request_ls_scan(1) # ls scan
    robot.pho_ls_wait_for_scan() # waiting for scan
    robot.pho_request_get_objects(1, 5) # get objects
    time.sleep(0.01) # sleep
    robot.pho_request_get_running_solution() # get running solution
    time.sleep(0.01)
    #robot.pho_request_move_to_position()
    # time.sleep(0.2)
    # robot.pho_request_stop_solution()
    # time.sleep(2)
    robot.pho_request_get_available_solution() # get available solution
    robot.close_connection()  #communication needs to be closed
    time.sleep(0.01)

def extract_object_coordinates(robot): # extract object coordinates [X,y,Z]
    try:
        # Replace 'objects' and 'coordinates' with actual attribute names from your response
        objects = robot.response_data.objects  # Example attribute; adjust accordingly
        
        if not objects:
            logging.info("No objects are found")
            return None

        # For simplicity, consider the first detected object
        first_object = objects[0]
        object_coords = first_object.coordinates  # Example attribute; adjust accordingly

        logging.info(f"Extracted Object Coordinates: {object_coords}")
        return object_coords

    except AttributeError:
        logging.error("Failed to extract object coordinates. Check the response data structure.")
        return None
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

def move_robot_to_position(robot, target_coords, tolerance=0.01, timeout=30):

    robot=RobotRequestResponseCommunication() # object is created
    servo=ServoX.servo_x() # calling servo function
    r.gripper("off") # setting gripper off
    try:
        start_time = time.time()
        while True:
            # Replace 'get_current_position' with the actual method to retrieve the robot's current position
            current_coords = robot.pho_get_current_position()
            distance = ((current_coords[0] - target_coords[0]) ** 2 +
                        (current_coords[1] - target_coords[1]) ** 2 +
                        (current_coords[2] - target_coords[2]) ** 2) ** 0.5
            if distance <= tolerance:
                logging.info(f"Robot reached target position: {current_coords}")
                break
            if time.time() - start_time > timeout:
                raise TimeoutError("Robot did not reach the target position in time.")
            time.sleep(0.5)
    except AttributeError:
        logging.error("The method 'get_current_position' does not exist in CommunicationLibrary.")
    except Exception as e:
        logging.error(f"An error occurred while moving the robot: {e}")
    return servo

# -------------------------------------------------------------------
#                      EXTRINSIC CALIBRATION
# -------------------------------------------------------------------
def calibration_extrinsic(): # function for extrinsic calibration
    robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
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
    robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
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


    #robot.pho_request_save_automatic_calibration()

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

