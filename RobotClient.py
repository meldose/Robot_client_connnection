#!/usr/bin/env python3

import CommunicationLibrary
import time
import json
import logging

CONTROLLER_IP = "192.168.1.5"
PORT = 11003


# def test_ls():
#     robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
#     robot.connect_to_server(CONTROLLER_IP,PORT)  # communication between VC and robot is created

#     robot.pho_request_start_solution(252)
#     robot.pho_request_ls_scan(1)
#     robot.pho_ls_wait_for_scan()
#     robot.pho_request_get_objects(1, 5)
#     time.sleep(0.3)
#     robot.pho_request_ls_get_vision_system_status(1)
#     time.sleep(0.3)
#     robot.pho_request_change_solution(253)
#     time.sleep(0.3)
#     robot.pho_request_ls_scan(1)
#     robot.pho_ls_wait_for_scan()
#     robot.pho_request_get_objects(1, 5)
#     time.sleep(0.3)
#     robot.pho_request_get_running_solution()
#     time.sleep(0.3)
#     # robot.pho_request_stop_solution()
#     # time.sleep(2)
#     robot.pho_request_get_available_solution()
#     robot.close_connection()  #communication needs to be closed
#     time.sleep(0.3)

def send_coordinates_to_robot(robot, coords):
    """
    Sends the specified coordinates to the robot controller.

    Args:
        robot: The robot communication object.
        coords (list): Coordinates to send [x, y, z].

    Returns:
        bool: True if the command was sent successfully, False otherwise.
    """
    try:
        x, y, z = coords
        logging.info(f"Sending move command to position: x={x}, y={y}, z={z}")
        robot.pho_request_move_to_position(x, y, z)  # Replace with actual method
        return True
    except AttributeError:
        logging.error("The method 'pho_request_move_to_position' does not exist in CommunicationLibrary.")
        return False
    except Exception as e:
        logging.error(f"Failed to send coordinates to robot: {e}")
        return False


def format_coordinates(coords_mm):
    """
    Converts coordinates from millimeters to meters.

    Args:
        coords_mm (list or tuple): Coordinates in millimeters [x, y, z].

    Returns:
        list: Coordinates in meters [x, y, z].
    """
    try:
        coords_m = [x / 1000.0 for x in coords_mm]
        return coords_m
    except TypeError:
        logging.error("Invalid type for coordinates. Expected list or tuple.")
        return None
    except Exception as e:
        logging.error(f"An error occurred while formatting coordinates: {e}")
        return None


def extract_object_coordinates(robot):
    """
    Extracts object coordinates from the robot's response data.

    Args:
        robot: The robot communication object.

    Returns:
        list: A list containing the [x, y, z] coordinates of the object.
              Returns None if no objects are detected.
    """
    try:
        # Assuming 'response_data.objects' contains a list of detected objects
        # and each object has 'coordinates' attribute as [x, y, z]
        objects = robot.response_data.objects  # Adjust based on actual attribute name

        if not objects:
            logging.info("No objects detected.")
            return None

        # For simplicity, consider the first detected object
        first_object = objects[0]
        object_coords = first_object.coordinates  # Adjust based on actual structure

        logging.info(f"Extracted Object Coordinates: {object_coords}")
        return object_coords

    except AttributeError:
        logging.error("Failed to extract object coordinates. Check the response data structure.")
        return None
    except Exception as e:
        logging.error(f"An error occurred while extracting object coordinates: {e}")
        return None


def test_ls():
    """
    Tests the LS (Laser Scan?) functionality of the robot.
    Extracts object coordinates from the camera and sends them to the robotic controller.
    """
    import CommunicationLibrary
    import time
    import logging

    # Configure logging for this function
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        handlers=[
            logging.StreamHandler()
        ]
    )

    robot = CommunicationLibrary.RobotRequestResponseCommunication()  # Create robot communication object
    try:
        logging.info(f"Connecting to robot at {CONTROLLER_IP}:{PORT}")
        robot.connect_to_server(CONTROLLER_IP, PORT)  # Establish communication

        logging.info("Starting solution 252")
        robot.pho_request_start_solution(252)

        logging.info("Initiating LS scan")
        robot.pho_request_ls_scan(1)  # Start LS scan with parameter 1
        robot.pho_ls_wait_for_scan()    # Wait for the scan to complete

        logging.info("Requesting objects detected in the scan")
        robot.pho_request_get_objects(1, 5)  # Get objects detected (parameters may vary)
        time.sleep(0.3)  # Short delay to ensure response is received

        logging.info("Retrieving vision system status")
        robot.pho_request_ls_get_vision_system_status(1)
        time.sleep(0.3)

        logging.info("Changing solution to 253")
        robot.pho_request_change_solution(253)
        time.sleep(0.3)

        logging.info("Initiating another LS scan")
        robot.pho_request_ls_scan(1)
        robot.pho_ls_wait_for_scan()

        logging.info("Requesting objects detected in the second scan")
        robot.pho_request_get_objects(1, 5)
        time.sleep(0.3)

        logging.info("Retrieving running solution")
        robot.pho_request_get_running_solution()
        time.sleep(0.3)

        logging.info("Retrieving available solutions")
        robot.pho_request_get_available_solution()

        # Extract object coordinates from the response
        object_coords = extract_object_coordinates(robot)
        if object_coords:
            # Format coordinates (e.g., convert from mm to m)
            formatted_coords = format_coordinates(object_coords)
            logging.info(f"Formatted Object Coordinates (meters): {formatted_coords}")

            # Send coordinates to the robot
            send_success = send_coordinates_to_robot(robot, formatted_coords)
            if send_success:
                logging.info("Coordinates sent successfully.")
            else:
                logging.error("Failed to send coordinates to the robot.")

            # Optional: Command the robot to move to the coordinates
            # move_success = move_robot_to_position(robot, formatted_coords)
            # if move_success:
            #     logging.info("Robot moved to the specified position successfully.")
            # else:
            #     logging.error("Robot failed to move to the specified position.")
        else:
            logging.warning("No object coordinates found in the response.")

    except CommunicationLibrary.ConnectionError as ce:
        logging.error(f"Connection failed: {ce}")
    except CommunicationLibrary.CommandError as cmd_err:
        logging.error(f"Command failed: {cmd_err}")
    except Exception as e:
        logging.error(f"An unexpected error occurred: {e}")
    finally:
        logging.info("Closing connection to the robot.")
        robot.close_connection()
        time.sleep(0.3)  # Short delay after closing the connection


def test_bps():
    robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
    robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

    robot.pho_request_start_solution(254)
    robot.pho_request_init(1, [3.14, 0.6, 1.13, 3.14, 0.6, 3.14], [3.14, 0.6, 1.13, 3.14, 0.6, 3.14])
    robot.pho_request_bps_scan(1)
    robot.pho_bps_wait_for_scan()
    time.sleep(2)
    robot.pho_request_trajectory(1)
    time.sleep(0.3)
    robot.pho_request_get_object(1) # command for getting the robot poses
    time.sleep(2)
    robot.pho_request_bsp_get_vision_system_status(1)
    time.sleep(2)
    robot.pho_request_pick_failed(1)
    time.sleep(2)
    robot.pho_request_change_scene_status(2)
    time.sleep(2)
    robot.pho_request_change_scene_status(1)
    time.sleep(2)
    robot.pho_request_get_running_solution()
    time.sleep(2)
    robot.pho_request_stop_solution()
    time.sleep(2)
    robot.pho_request_get_available_solution()

    robot.close_connection()  # communication needs to be closed
    time.sleep(0.3)

    # print(robot.response_data.trajectory_data)
    # print(robot.response_data.trajectory_data[1])
    # print(robot.response_data.trajectory_data[2])
    print(robot.response_data.gripper_command)
    # Print each slice to verify
    for i, segment in enumerate(robot.response_data.trajectory_data):
        print(f"Segment {i}:")
        print(segment)


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

