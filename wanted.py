

import CommunicationLibrary # importing communication library
import time # importing time
import json # importing json
import logging #    importing logging
from neurapy.robot import Robot
from ruckig import InputParameter, OutputParameter, Result, Ruckig 
from CommunicationLibrary import ServoJ, ResponseData, RobotRequestResponseCommunication

r=Robot()

CONTROLLER_IP = "192.168.1.5" # IP address of the controller
PORT = 11003 # port number

# robot_re = RobotRequestResponseCommunication()

# response_data = ResponseData() 

def test_ls(): # test ls
    robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
    robot.connect_to_server(CONTROLLER_IP,PORT)  # communication between VC and robot is created

    robot.pho_request_start_solution(252) # start solution
    robot.pho_request_ls_scan(1) # ls scanS
    robot.pho_ls_wait_for_scan() # wait for scan
    robot.pho_request_get_objects(1, 5) # get objects
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

def move_robot_to_position(robot, target_coords, tolerance=0.01, timeout=30): # function for moving robot to position

    servo=ServoJ()

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


# def move_robot_to_position(robot, target_coords, tolerance=0.01, timeout=30):

def servo_j(): # defining function for servoJ
#Switch to external servo mode

    # x = message[0]
    # y = message[1]
    # z = message[2]
    # a = message[3]
    # b = message[4]
    # c = message[5]
    # d = message[6]

    r.activate_servo_interface('position') # activating the servo interface
    dof = 6 # setting the DOF as 6 
    otg = Ruckig(dof, 0.001)  # DoFs, control cycle

    inp = InputParameter(dof) # setting the input parameter
    out = OutputParameter(dof) # setting the output parameter

    inp.current_position = r.get_current_joint_angles() # getting the current joint angles
    inp.current_velocity = [0.]*dof
    inp.current_acceleration = [0.]*dof

    inp.target_position = [1.1625650370244778, -0.5774947959093657, -1.6300017754314295, 1.9807964651163987, 1.5676122261006906, 0.636066807616557] # target positon
    #inp.target_position = [x,y,z,a,b,c,d]
    inp.max_velocity = [0.5]*dof # setting up the maximum velocity 
    inp.max_acceleration = [3]*dof # setting up the maximum acceleration


    inp = InputParameter(dof) # setting the input parameter
    out = OutputParameter(dof) # setting the ouput parameters 
    inp.current_position = r.get_current_joint_angles() # getting the current joint angles
    inp.current_velocity = [0.]*dof # setting the current velocity as zero
    inp.current_acceleration = [0.]*dof # setting the current acceleration as zero

    inp.target_position = [1.1625650370244778, -0.5774947959093657, -1.6300017754314295, 1.9807964651163987, 1.5676122261006906, 0.636066807616557] # providing the target position
    inp.target_acceleration = [0.]*dof # setting the target acceleration as zero.
    r.gripper("on") # setting the gripper in On position.

    inp.max_velocity = [0.5]*dof # defining the maximum velocity
    inp.max_acceleration = [3]*dof # defining the maximum acceleration

    inp.max_jerk = [10.]*dof
    res = Result.Working

    while res == Result.Working:
        error_code = 0

        res = otg.update(inp, out)

        position = out.new_position # setting the new position 
        velocity = out.new_velocity # setting the new velocity
        acceleration = out.new_acceleration # setting the new acceleration 

        error_code = r.servo_j(position, velocity, acceleration) # passing the error code variable with having servo_j function having position, velocity and acceleration.
        print(error_code) # checking if the error is there or not 
        scaling_factor = r.get_servo_trajectory_scaling_factor() # getting the servo trajectory scaling factors.
        out.pass_to_input(inp)
        time.sleep(0.001) # setting the time sleep to 0.001 seconds

    r.deactivate_servo_interface() # deactivating the servo interface

    r.stop() # stopped the robot

servo_j() # calling the servo_j function
r.gripper("off") # setting gripper off


    
    # function for moving robot to position
    # def servo_j(): # defining function for servoJ
    #     #Switch to external servo mode
    #     r.activate_servo_interface('position') # activating the servo interface
    #     dof = 6 # setting the DOF as 6 
    #     otg = Ruckig(dof, 0.001)  # DoFs, control cycle

    #     inp = InputParameter(dof) # setting the input parameter
    #     out = OutputParameter(dof) # setting the output parameter
    
    #     inp.current_position = r.get_current_joint_angles() # getting the current joint angles
    #     inp.current_velocity = [0.]*dof
    #     inp.current_acceleration = [0.]*dof
    
    #     inp.target_position = [1.1625650370244778, -0.5774947959093657, -1.6300017754314295, 1.9807964651163987, 1.5676122261006906, 0.636066807616557] # target positon
    
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
    
    #     r.stop() # stopped the robot

    # #servo_j() # calling the servo_j function
    # r.gripper("off") # setting gripper off


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


