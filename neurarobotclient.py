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


# def test_ls():
#     robot = CommunicationLibrary.RobotRequestResponseCommunication()
#     try:
#         logging.info(f"Connecting to robot at {CONTROLLER_IP}:{PORT}")
#         robot.connect_to_server(CONTROLLER_IP, PORT)

#         robot.pho_request_start_solution(252)
#         robot.pho_request_ls_scan(1)
#         robot.pho_ls_wait_for_scan()
#         robot.pho_request_get_objects(1, 5)
#         robot.pho_send_request()
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


import time
import logging
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
from neurapy.robot import Robot
from ruckig import InputParameter, OutputParameter, Result, Ruckig
import CommunicationLibrary

CONTROLLER_IP = "192.168.1.5"
PORT = 11003

r = Robot()


def test_ls():
    """Retrieve object position and orientation from the vision system."""
    robot = CommunicationLibrary.RobotRequestResponseCommunication()
    try:
        logging.info(f"Connecting to robot at {CONTROLLER_IP}:{PORT}")
        robot.connect_to_server(CONTROLLER_IP, PORT)

        robot.pho_request_start_solution(252)
        robot.pho_request_ls_scan(1)
        robot.pho_ls_wait_for_scan()
        robot.pho_request_get_objects(1, 5)
        time.sleep(0.1)

        object_pose = extract_object_pose(robot)
        if object_pose:
            formatted_pose = format_pose(object_pose)
            if formatted_pose:
                logging.info(f"Formatted Object Pose: {formatted_pose}")
                return formatted_pose
            else:
                logging.warning("Failed to format object pose.")
                return None
        else:
            logging.warning("No object pose found in the response.")
            return None

    except Exception as e:
        logging.error(f"Error in test_ls: {e}")
        return None
    finally:
        robot.close_connection()


def extract_object_pose(robot):
    """Extract both position and quaternion orientation from the vision system response."""
    try:
        objects = robot.response_data.objects
        if not objects:
            logging.info("No objects found.")
            return None
        
        object_data = objects[0]
        position = object_data.coordinates  # Assuming [x, y, z] in mm
        quaternion = object_data.quaternion  # Assuming [w, x, y, z]

        return position, quaternion

    except AttributeError:
        logging.error("Invalid response structure.")
        return None
    except Exception as e:
        logging.error(f"Error extracting object pose: {e}")
        return None

def format_pose(pose_data):
    """Convert mm to meters and transform quaternion to Euler angles."""
    try:
        position_mm, quaternion = pose_data
        position_m = [x / 1000.0 for x in position_mm]  # Convert to meters

        # Convert Quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw)
        r = R.from_quat([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
        euler_angles = r.as_euler('xyz', degrees=True)

        return position_m + list(euler_angles)

    except TypeError:
        logging.error("Invalid type for pose data.")
        return None


def servo_x(target_pose):
    """Move the robot to the target pose using servo control."""
    r.activate_servo_interface('position')
    cart_pose_length = 6  # Adjusted for position + Euler angles
    otg = Ruckig(cart_pose_length, 0.001)
    inp = InputParameter(cart_pose_length)
    out = OutputParameter(cart_pose_length)

    inp.current_position = r.get_current_cartesian_pose()[:6]  # Extract [x, y, z, roll, pitch, yaw]
    inp.current_velocity = [0.0] * cart_pose_length
    inp.current_acceleration = [0.0] * cart_pose_length

    inp.target_position = target_pose  # Now includes Euler angles
    inp.target_velocity = [0.0] * cart_pose_length
    inp.target_acceleration = [0.0] * cart_pose_length

    inp.max_velocity = [0.5] * cart_pose_length
    inp.max_acceleration = [3] * cart_pose_length
    inp.max_jerk = [10.0] * cart_pose_length

    res = Result.Working
    servox_proportional_gain = 25
    velocity = [0.0] * cart_pose_length
    acceleration = [0.0] * cart_pose_length

    while res == Result.Working:
        res = otg.update(inp, out)
        position = out.new_position

        for i in range(cart_pose_length):
            velocity[i] = out.new_velocity[i]
            acceleration[i] = out.new_acceleration[i]

        error_code = r.servo_x(position, velocity, acceleration, servox_proportional_gain)
        print(f"Error Code: {error_code}")

        out.pass_to_input(inp)
        time.sleep(0.001)

    r.deactivate_servo_interface()
    r.stop()


# Main Execution
object_pose = test_ls()
if object_pose:
    servo_x(object_pose)


