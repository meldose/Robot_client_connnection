# #!/usr/bin/env python3

# import CommunicationLibrary
# import time
# import json

# CONTROLLER_IP = "192.168.1.5"
# PORT = 11003


# def test_ls():
#     robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
#     robot.connect_to_server(CONTROLLER_IP,PORT)  # communication between VC and robot is created

#     robot.pho_request_start_solution(252)
#     robot.pho_request_ls_scan(1)
#     robot.pho_ls_wait_for_scan()
#     robot.pho_request_get_objects(1, 5)
#     time.sleep(2)
#     robot.pho_request_ls_get_vision_system_status(1)
#     time.sleep(2)
#     robot.pho_request_change_solution(253)
#     time.sleep(2)
#     robot.pho_request_ls_scan(1)
#     robot.pho_ls_wait_for_scan()
#     robot.pho_request_get_objects(1, 5)
#     time.sleep(2)
#     robot.pho_request_get_running_solution()
#     time.sleep(2)
#     robot.pho_request_stop_solution()
#     time.sleep(2)
#     robot.pho_request_get_available_solution()

#     robot.close_connection()  #communication needs to be closed
#     time.sleep(2)


# def test_bps():
#     robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
#     robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

#     robot.pho_request_start_solution(254)
#     robot.pho_request_init(1, [3.14, 0.6, 1.13, 3.14, 0.6, 3.14], [3.14, 0.6, 1.13, 3.14, 0.6, 3.14])
#     robot.pho_request_bps_scan(1)
#     robot.pho_bps_wait_for_scan()
#     time.sleep(2)
#     robot.pho_request_trajectory(1)
#     time.sleep(1)
#     robot.pho_request_get_object(1)
#     time.sleep(2)
#     robot.pho_request_bsp_get_vision_system_status(1)
#     time.sleep(2)
#     robot.pho_request_pick_failed(1)
#     time.sleep(2)
#     robot.pho_request_change_scene_status(2)
#     time.sleep(2)
#     robot.pho_request_change_scene_status(1)
#     time.sleep(2)
#     robot.pho_request_get_running_solution()
#     time.sleep(2)
#     robot.pho_request_stop_solution()
#     time.sleep(2)
#     robot.pho_request_get_available_solution()

#     robot.close_connection()  # communication needs to be closed
#     time.sleep(1)

#     # print(robot.response_data.trajectory_data)
#     # print(robot.response_data.trajectory_data[1])
#     # print(robot.response_data.trajectory_data[2])
#     print(robot.response_data.gripper_command)
#     # Print each slice to verify
#     for i, segment in enumerate(robot.response_data.trajectory_data):
#         print(f"Segment {i}:")
#         print(segment)


# def calibration_extrinsic():
#     robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
#     robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

#     robot.pho_request_start_automatic_calibration(6,1)
#     # Load the JSON data
#     file_path = 'extrinsic_calib_points.json'
#     json_data = load_json_file(file_path)

#     # add 9 calibration point
#     for point in json_data:
#         translation_mm = point["translation"]
#         quaternion = point["quaternion"]
#         translation_m = [x * 1000 for x in translation_mm] # mm to m
#         tool_pose = translation_m + quaternion

#         robot.pho_request_add_calibration_point(tool_pose)
#         time.sleep(2)

#     robot.pho_request_save_automatic_calibration()
#     time.sleep(2)
#     robot.pho_request_stop_automatic_calibration()


# def calibration_handeye():
#     robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
#     robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

#     robot.pho_request_start_automatic_calibration(6, 2)
#     # Load the JSON data
#     file_path = 'handeye_calib_points.json'
#     json_data = load_json_file(file_path)

#     # add 9 calibration point
#     for point in json_data:
#         translation_mm = point["translation"]
#         quaternion = point["quaternion"]
#         translation_m = [x * 1000 for x in translation_mm]  # mm to m
#         tool_pose = translation_m + quaternion

#         robot.pho_request_add_calibration_point(tool_pose)
#         time.sleep(2)

#     #robot.pho_request_save_automatic_calibration()
#     time.sleep(2)
#     robot.pho_request_stop_automatic_calibration()


# # Function to load JSON data from a file
# def load_json_file(file_path):
#     with open(file_path, 'r') as file:
#         data = json.load(file)
#     return data


# if __name__ == '__main__':

#     #calibration_handeye()
#     #calibration_extrinsic()
#     #test_ls()
#     test_bps()

#     #while True:
#         #test_ls()
#         #test_bps()


#!/usr/bin/env python3

import CommunicationLibrary
import time
import json

CONTROLLER_IP = "192.168.1.5"
PORT = 11003


def connect_robot():
    try:
        robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
        robot.connect_to_server(CONTROLLER_IP, PORT)  # communication with robot
        return robot
    except Exception as e:
        print(f"Error connecting to the robot: {e}")
        return None


def test_ls():
    robot = connect_robot()
    if robot is None:
        return

    try:
        robot.pho_request_start_solution(252)
        robot.pho_request_ls_scan(1)

        # Wait for the scan to complete
        scan_complete = False
        while not scan_complete:
            scan_complete = robot.pho_ls_wait_for_scan()
            time.sleep(0.5)  # Check every 0.5 seconds

        # Check if objects are detected
        if not robot.response_data.objects:
            print("No objects detected.")
        else:
            # Print object coordinates (X, Y, Z)
            for obj in robot.response_data.objects:
                print(f"Object ID: {obj.id}")
                # Assuming position is a property of the object (modify based on actual structure)
                print(f"Position: X={obj.position.x}, Y={obj.position.y}, Z={obj.position.z}")

        # Optionally, print additional information for debugging
        print("Response Data: ", robot.response_data)

        # Request available solution
        robot.pho_request_get_available_solution()

    finally:
        robot.close_connection()  # Ensure connection is closed
        time.sleep(2)


def test_bps():
    robot = connect_robot()
    if robot is None:
        return

    try:
        robot.pho_request_start_solution(254)
        robot.pho_request_init(1, [3.14, 0.6, 1.13, 3.14, 0.6, 3.14], [3.14, 0.6, 1.13, 3.14, 0.6, 3.14])
        robot.pho_request_bps_scan(1)

        # Wait for the scan to complete
        scan_complete = False
        while not scan_complete:
            scan_complete = robot.pho_bps_wait_for_scan()
            time.sleep(0.5)  # Check every 0.5 seconds

        # Check if objects are detected
        if not robot.response_data.objects:
            print("No objects detected.")
        else:
            # Print object coordinates (X, Y, Z)
            for obj in robot.response_data.objects:
                print(f"Object ID: {obj.id}")
                print(f"Position: X={obj.position.x}, Y={obj.position.y}, Z={obj.position.z}")

        # Request available solution
        robot.pho_request_get_available_solution()

        # Optionally, print additional information for debugging
        print("Response Data: ", robot.response_data)

    finally:
        robot.close_connection()  # Ensure connection is closed
        time.sleep(2)


def calibration_extrinsic():
    robot = connect_robot()
    if robot is None:
        return

    try:
        robot.pho_request_start_automatic_calibration(6, 1)

        # Load the JSON data for calibration points
        file_path = 'extrinsic_calib_points.json'
        json_data = load_json_file(file_path)

        # Add calibration points
        for point in json_data:
            translation_mm = point["translation"]
            quaternion = point["quaternion"]
            translation_m = [x * 1000 for x in translation_mm]  # mm to m
            tool_pose = translation_m + quaternion
            robot.pho_request_add_calibration_point(tool_pose)
            time.sleep(2)

        robot.pho_request_save_automatic_calibration()
        time.sleep(2)
        robot.pho_request_stop_automatic_calibration()

    finally:
        robot.close_connection()  # Ensure connection is closed
        time.sleep(2)


def calibration_handeye():
    robot = connect_robot()
    if robot is None:
        return

    try:
        robot.pho_request_start_automatic_calibration(6, 2)

        # Load the JSON data for hand-eye calibration points
        file_path = 'handeye_calib_points.json'
        json_data = load_json_file(file_path)

        # Add calibration points
        for point in json_data:
            translation_mm = point["translation"]
            quaternion = point["quaternion"]
            translation_m = [x * 1000 for x in translation_mm]  # mm to m
            tool_pose = translation_m + quaternion
            robot.pho_request_add_calibration_point(tool_pose)
            time.sleep(2)

        robot.pho_request_stop_automatic_calibration()

    finally:
        robot.close_connection()  # Ensure connection is closed
        time.sleep(2)


# Function to load JSON data from a file
def load_json_file(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data


if __name__ == '__main__':
    test_ls()
    # test_bps()

    while True:
        test_ls()
        # test_bps()
