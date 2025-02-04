# import time
# import logging
# import copy
# import json
# from neurapy.robot import Robot
# from ruckig import InputParameter, OutputParameter, Result, Ruckig
# import CommunicationLibrary

# CONTROLLER_IP = "192.168.1.5"
# PORT = 11003

# r = Robot()
# r.gripper("on")

# def test_ls():
#     robot = CommunicationLibrary.RobotRequestResponseCommunication()
#     try:
#         logging.info(f"Connecting to robot at {CONTROLLER_IP}:{PORT}")
#         robot.connect_to_server(CONTROLLER_IP, PORT)

#         robot.pho_request_start_solution(252)
#         robot.pho_request_ls_scan(1)
#         robot.pho_ls_wait_for_scan()
#         robot.pho_request_get_objects(1, 5)
#         time.sleep(0.1)

#         object_coords = extract_object_coordinates(robot)
#         if object_coords:
#             formatted_coords = format_coordinates(object_coords)
#             if formatted_coords:
#                 logging.info(f"Formatted Object Coordinates: {formatted_coords}")
#                 return formatted_coords
#             else:
#                 logging.warning("Failed to format object coordinates.")
#                 return None
#         else:
#             logging.warning("No object coordinates found in the response.")
#             return None

#     except Exception as e:
#         logging.error(f"Error in test_ls: {e}")
#         return None
#     finally:
#         robot.close_connection()

# def extract_object_coordinates(robot):
#     try:
#         objects = robot.response_data.objects
#         if not objects:
#             logging.info("No objects found.")
#             return None
#         return objects[0].coordinates
#     except AttributeError:
#         logging.error("Invalid response structure.")
#         return None
#     except Exception as e:
#         logging.error(f"Error extracting object coordinates: {e}")
#         return None

# def format_coordinates(coords_mm):
#     try:
#         return [x / 1000.0 for x in coords_mm]
#     except TypeError:
#         logging.error("Invalid type for coordinates.")
#         return None

# def servo_x(target_position):
#     r.activate_servo_interface('position')
#     cart_pose_length = 7
#     otg = Ruckig(cart_pose_length, 0.001)
#     inp = InputParameter(cart_pose_length)
#     out = OutputParameter(cart_pose_length)

#     inp.current_position = r.get_current_cartesian_pose()
#     inp.current_velocity = [0.0] * cart_pose_length
#     inp.current_acceleration = [0.0] * cart_pose_length

#     if len(target_position) == 3:
#         target = inp.current_position[:3]
#         target[:3] = target_position
#     else:
#         target = target_position

#     inp.target_position = target
#     inp.target_velocity = [0.0] * cart_pose_length
#     inp.target_acceleration = [0.0] * cart_pose_length

#     inp.max_velocity = [0.5] * cart_pose_length
#     inp.max_acceleration = [3] * cart_pose_length
#     inp.max_jerk = [10.0] * cart_pose_length

#     res = Result.Working
#     servox_proportional_gain = 25
#     velocity = [0.0] * 6
#     acceleration = [0.0] * 6

#     while res == Result.Working:
#         res = otg.update(inp, out)
#         position = out.new_position
        
#         for i in range(3):
#             velocity[i] = out.new_velocity[i]
#             acceleration[i] = out.new_acceleration[i]
        
#         error_code = r.servo_x(position, velocity, acceleration, servox_proportional_gain)
#         print(f"Error Code: {error_code}")
#         out.pass_to_input(inp)
#         time.sleep(0.001)

#     r.deactivate_servo_interface()
#     r.stop()

# # Main Execution
# object_coords = test_ls()
# if object_coords:
#     servo_x(object_coords)
# r.gripper("off")


import time
import logging
import copy
import json
from neurapy.robot import Robot
from ruckig import InputParameter, OutputParameter, Result, Ruckig
import CommunicationLibrary

CONTROLLER_IP = "192.168.1.5"
PORT = 11003

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

r = Robot()
# r.gripper("on")

def test_ls():
    robot = CommunicationLibrary.RobotRequestResponseCommunication()
    try:
        logging.info(f"Connecting to robot at {CONTROLLER_IP}:{PORT}")
        robot.connect_to_server(CONTROLLER_IP, PORT)

        robot.pho_request_start_solution(252)
        robot.pho_request_ls_scan(1)
        robot.pho_ls_wait_for_scan()
        robot.pho_request_get_objects(1, 5)
        time.sleep(0.1)

        object_coords = extract_object_coordinates(robot)
        if object_coords:
            formatted_coords = format_coordinates(object_coords)
            if formatted_coords:
                logging.info(f"Formatted Object Coordinates: {formatted_coords}")
                return formatted_coords
            else:
                logging.warning("Failed to format object coordinates.")
                return None
        else:
            logging.warning("No object coordinates found in the response.")
            return None

    except Exception as e:
        logging.error(f"Error in test_ls: {e}")
        return None
    finally:
        robot.close_connection()

def extract_object_coordinates(robot):
    try:
        if not hasattr(robot, 'response_data') or not hasattr(robot.response_data, 'objects'):
            logging.error("Invalid response structure: Missing attributes.")
            return None

        objects = robot.response_data.objects
        if not objects:
            logging.info("No objects found.")
            return None
        return objects[0].coordinates
    except Exception as e:
        logging.error(f"Error extracting object coordinates: {e}")
        return None

def format_coordinates(coords_mm):
    try:
        return [x / 1000.0 for x in coords_mm]
    except TypeError:
        logging.error("Invalid type for coordinates.")
        return None

def servo_x(target_position):
    r.activate_servo_interface('position')
    cart_pose_length = 7
    otg = Ruckig(cart_pose_length, 0.001)
    inp = InputParameter(cart_pose_length)
    out = OutputParameter(cart_pose_length)

    inp.current_position = r.get_current_cartesian_pose()
    inp.current_velocity = [0.0] * cart_pose_length
    inp.current_acceleration = [0.0] * cart_pose_length

    if len(target_position) == 3:
        target = inp.current_position.copy()
        target[:3] = target_position
    else:
        target = target_position

    inp.target_position = target
    inp.target_velocity = [0.0] * cart_pose_length
    inp.target_acceleration = [0.0] * cart_pose_length

    inp.max_velocity = [0.5] * cart_pose_length
    inp.max_acceleration = [3] * cart_pose_length
    inp.max_jerk = [10.0] * cart_pose_length

    res = Result.Working
    servox_proportional_gain = 25
    velocity = [0.0] * 6
    acceleration = [0.0] * 6

    max_iterations = 10000
    iteration = 0

    while res == Result.Working and iteration < max_iterations:
        res = otg.update(inp, out)
        position = out.new_position

        for i in range(3):
            velocity[i] = out.new_velocity[i]
            acceleration[i] = out.new_acceleration[i]

        error_code = r.servo_x(position, velocity, acceleration, servox_proportional_gain)
        logging.info(f"Error Code: {error_code}")

        out.pass_to_input(inp)
        time.sleep(0.001)
        iteration += 1

    if iteration >= max_iterations:
        logging.warning("servo_x() reached maximum iterations, stopping.")

    r.deactivate_servo_interface()
    r.stop()

# ==============================
#       MAIN EXECUTION LOOP
# ==============================

MAX_NO_OBJECTS_COUNT = 5  # Stop after 5 consecutive failures to detect objects
no_object_counter = 0

try:
    while no_object_counter < MAX_NO_OBJECTS_COUNT:
        logging.info("Starting a new cycle...")

        object_coords = test_ls()
        if object_coords:
            no_object_counter = 0  # Reset counter when objects are detected
            servo_x(object_coords)
        else:
            no_object_counter += 1
            logging.warning(f"No objects detected ({no_object_counter}/{MAX_NO_OBJECTS_COUNT})")

        time.sleep(1)  # Prevent excessive CPU usage

except KeyboardInterrupt:
    logging.info("User requested termination. Stopping the loop.")

finally:
    r.gripper("off")
    logging.info("Robot operations stopped.")

# Main Execution
object_coords = test_ls()
if object_coords:
    servo_x(object_coords)
# r.gripper("off")

