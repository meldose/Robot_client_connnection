
import socket # importing socket
from copy import deepcopy # importing copy
import struct # importing struct
import math # importing math
import numpy as np # importing numpy
# from StateServer import get_joint_state, get_tool_pose, init_joint_state, base_quat
# from StateServer import init_joint_state, base_quat
import time # importing times
import logging # importing logging
from ruckig import InputParameter, OutputParameter, Result, Ruckig  # importing ruckig . output parameter,InputParameter and Result
from neurapy.robot import Robot # importing robot
r=Robot() # defining the robot

BRAND_IDENTIFICATION = "ABB_IRB/1.8.0XXXXXXXXXXX"  # "DOOSAN/1.7.0_XXXXXXXXXXX" "UNIVERSAL_ROBOTS/v1.8.0X" # "KAWASAKI/1.8.0XXXXXXXXXX" # "KUKA_SUNRISE/1.8.0XXXXXX" # "KUKA_KRC/1.8.0XXXXX" #  "BPS_EXT_DEVICE/1.8.0XXXX"  "KUKA_KRC/1.8.0XXXXX"  #  "ABB_IRB/1.8.0XXXXXXXXXXX"
BRAND_IDENTIFICATION_SERVER = "ABB_IRB/1.8.0XXXXXXXXXXX"

DEG2RAD = math.pi / 180 # converting degrees to radians


class OperationType: # defining operation type
    PHO_TRAJECTORY_CNT = 0  # defining operation type    
    PHO_TRAJECTORY_FINE = 1 # defining operation type
    PHO_GRIPPER = 2 # defining operation type
    PHO_ERROR = 3 # defining operation type
    PHO_INFO = 4 # defining operation type
    PHO_OBJECT_POSE = 5 # defining operation type


PHO_SCAN_BPS_REQUEST = 1
PHO_SCAN_LS_REQUEST = 19
PHO_TRAJECTORY_REQUEST = 2
PHO_INIT_REQUEST = 4
PHO_ADD_CAL_POINT_REQUEST = 5
PHO_PICK_FAILED_REQUEST = 7
PHO_GET_OBJECT_BPS_REQUEST = 8
PHO_CHANGE_SOLUTION_REQUEST = 9
PHO_START_SOLUTION_REQUEST = 10
PHO_STOP_SOLUTION_REQUEST = 11
PHO_GET_RUNNING_SOLUTION_REQUEST = 12
PHO_GET_AVAILABLE_SOLUTION_REQUEST = 13
PHO_CHANGE_SCENE_STATE_REQUEST = 15
PHO_GET_OBJECT_LS_REQUEST = 20
PHO_GET_VISION_SYSTEM_BPS_REQUEST = 21
PHO_GET_VISION_SYSTEM_LS_REQUEST = 22
PHO_START_AUTO_CAL_REQUEST = 25
PHO_STOP_AUTO_CAL_REQUEST = 26
PHO_SAVE_AUTO_CAL_REQUEST = 27

request_name = {
    PHO_SCAN_BPS_REQUEST: "SCAN",
    PHO_SCAN_LS_REQUEST: "SCAN",
    PHO_TRAJECTORY_REQUEST: "TRAJECTORY",
    PHO_INIT_REQUEST: "INIT",
    PHO_ADD_CAL_POINT_REQUEST: "ADD CALIBRATION POINT",
    PHO_PICK_FAILED_REQUEST: "PICK FAILED",
    PHO_GET_OBJECT_BPS_REQUEST: "GET OBJECT",
    PHO_CHANGE_SOLUTION_REQUEST: "CHANGE SOLUTION",
    PHO_START_SOLUTION_REQUEST: "START SOLUTION",
    PHO_STOP_SOLUTION_REQUEST: "STOP SOLUTION",
    PHO_GET_RUNNING_SOLUTION_REQUEST: "GET RUNNING SOLUTION",
    PHO_GET_AVAILABLE_SOLUTION_REQUEST: "GET AVAILABLE SOLUTION",
    PHO_CHANGE_SCENE_STATE_REQUEST: "CHANGE SCENE",
    PHO_GET_OBJECT_LS_REQUEST: "GET OBJECTS",
    PHO_GET_VISION_SYSTEM_BPS_REQUEST: "GET VISION SYSTEM",
    PHO_GET_VISION_SYSTEM_LS_REQUEST: "GET VISION SYSTEM",
    PHO_START_AUTO_CAL_REQUEST: "START AUTOMATIC CALIBRATION",
    PHO_STOP_AUTO_CAL_REQUEST: "STOP AUTOMATIC CALIBRATION",
    PHO_SAVE_AUTO_CAL_REQUEST: "SAVE AUTOMATIC CALIBRATION",
}

# STATE SERVER Requests
JOINT_STATE_TYPE = 1
TOOL_POSE_TYPE = 2

# sizes
HEADER_SIZE = 12
SUBHEADER_SIZE = 12
PACKET_SIZE = 4
OBJECT_POSE_SIZE = 28

# Photoneo header
PHO_HEADER = [80, 0, 0, 0, 72, 0, 0, 0, 79, 0, 0, 0]  # P, H, O

# class ServoJ: # defining servoJ
#     def __init__(self, robot): # initializing the robot
#         self.robot = robot # setting the robot

#     def servo_j(self,message):

#         message = [x / 1000 for x in message]
        
#         x = message[0] 
#         y = message[1] 
#         z = message[2] 
#         a = message[3] 
#         b = message[4] 
#         c = message[5] 
#         d = message[6] 

        

        #   print(message)
        
        #   new_message = [x, y,z,d, a, b, c] # added new order for quaternion values
        #   print(new_message)


#         r.activate_servo_interface('position') # activating the servo interface
#         dof = 6 # setting the DOF as 6 
#         otg = Ruckig(dof, 0.001)  # DoFs, control cycle

#         inp = InputParameter(dof) # setting the input parameter
#         out = OutputParameter(dof) # setting the output parameter
    
#         inp.current_position = r.get_current_joint_angles() # getting the current joint angles
#         inp.current_velocity = [0.]*dof
#         inp.current_acceleration = [0.]*dof
    
#         # for home position ####
#         inp.target_position = [x,y,z,a,b,c,d]
#         inp.max_velocity = [0.5]*dof # setting up the maximum velocity 
#         inp.max_acceleration = [3]*dof # setting up the maximum acceleration
    
#         inp = InputParameter(dof) # setting the input parameters
#         out = OutputParameter(dof) # setting the ouput parameters 
#         inp.current_position = r.get_current_joint_angles() # getting the current joint angles
#         inp.current_velocity = [0.]*dof # setting the current velocity as zero
#         inp.current_acceleration = [0.]*dof # setting the current acceleration as zero
    
#         inp.target_position = [x,y,z,a,b,c,d]
#         inp.target_acceleration = [0.]*dof # setting the target acceleration as zero.
#         r.gripper("on") # setting the gripper in On position.
    
#         inp.max_velocity = [0.5]*dof # defining the maximum velocity
#         inp.max_acceleration = [3]*dof # defining the maximum acceleration

#         inp.max_jerk = [10.]*dof
#         res = Result.Working
    
#         while res == Result.Working:
#             error_code = 0

#             res = otg.update(inp, out)

#             position = out.new_position # setting the new position 
#             velocity = out.new_velocity # setting the new velocity
#             acceleration = out.new_acceleration # setting the new acceleration 

#             error_code = r.servo_j(position, velocity, acceleration) # passing the error code variable with having servo_j function having position, velocity and acceleration.
#             #print(error_code) # checking if the error is there or not 
#             scaling_factor = r.get_servo_trajectory_scaling_factor() # getting the servo trajectory scaling factors.
#             out.pass_to_input(inp)
#             time.sleep(0.001) # setting the time sleep to 0.001 seconds

#         r.deactivate_servo_interface() # deactivating the servo interface
    
#         r.stop() # stopped the robot

# ServoJ(robot=r).servo_j() # line for initial robot movement
# r.gripper("off") # setting gripper off

import copy

class ServoX:

    def __init__(self,robot):
        self.robot = robot


    def servo_x(self,message,*args,**kwargs):
        
        message = [x/1000 for x in message]
        
        x = message[0] 
        y = message[1] 
        z = message[2] 
        a = message[3] 
        b = message[4] 
        c = message[5] 
        d = message[6] 
        
        print(message)
        
        new_message = [x, y,z,d, a, b, c] # added new order for quaternion values
        print(new_message)


        # r = self.robot

        #Switch to external servo mode
        r.activate_servo_interface('position')

        cart_pose_length = 7 #X,Y,Z,qw,qx,qy,qz 

        otg = Ruckig(cart_pose_length, 0.001)  # control cycle
        inp = InputParameter(cart_pose_length) # setting the inputparameter with cart pose length
        out = OutputParameter(cart_pose_length) # setting the outputparmeter with cart pose length

        inp.current_position = r.get_current_cartesian_pose()
        # inp.target_position = [-0.522, -0.315, 0.120,-3.02,-0.06,1.41] # providing the target position
        # inp.current_velocity = [0.]*cart_pose_length # mutliplying the initila velocity with cart pose lenght 
        # inp.current_acceleration = [0.]*cart_pose_length # mutliplying the current acceleration with cart pose length

        target = copy.deepcopy(inp.current_position) # copying the current position of the robot 
        inp.target_velocity = [0.]*cart_pose_length # defning the target velocity
        inp.target_acceleration = [0.]*cart_pose_length # definng the target acceleration

        target = copy.deepcopy(inp.current_position) # copying the current position of the robot 
        inp.target_position = new_message # initating the target position
        inp.target_velocity = [0.]*cart_pose_length # defning the target velocity
        inp.target_acceleration = [0.]*cart_pose_length # definng the target acceleration
        # inp.target_position = [x,y,z,a,b,c,d] 
        # inp.target_position = [-0.522, -0.315, 0.120,-3.02,-0.06,1.41]
        # target[0] += 0.2 # Move 200mm in X direction

        inp.max_velocity = [0.3]*cart_pose_length # setting the maximum velocity with 0.5 times the cart pose lenght 
        inp.max_acceleration = [1]*cart_pose_length #se tting the max acceleration with 3 times the cart pose length
        inp.max_jerk = [5.]*cart_pose_length # setting the jerk values

        servox_proportional_gain = 25 # setting the servox propotional gain as 25

        velocity = [0.] * 6 #Since ruckig does not provide rotational velocity if quaternion is input, we can send 0 rotational feedforward velocity
        acceleration = [0.] * 6 #Since ruckig does not provide rotational acceleration if quaternion is input, we can send 0 rotational feedforward acceleration
        
        res=Result.Working

        while res == Result.Working:
            error_code = 0

            res = otg.update(inp, out)

            position = out.new_position

            for i in range(0,3): # Updating target translation velocity and accelerations
                velocity[i] = out.new_velocity[i]
                acceleration[i] = out.new_acceleration[i]
            
            error_code = r.servo_x(position, velocity, acceleration, servox_proportional_gain)
            # print(error_code) # checking if there is an error or not 
            scaling_factor = r.get_servo_trajectory_scaling_factor()
            out.pass_to_input(inp)
            time.sleep(0.001) # setting time 

        r.deactivate_servo_interface() # deactivating the servo interface

        r.stop() # stopping the robot
        
# ServoX(robot=r).servo_x()
r.set_mode("Automatic")
r.gripper("off")
r.move_joint("P16")
r.set_mode("Teach")
r.gripper("on")

        
class ResponseHeader: # class used for storing data
    def __init__(self, request_id, sub_headers): # initializing the class
        self.request_id = request_id # setting the request id
        self.sub_headers = sub_headers # setting the sub headers


class ResponseData: # class used for storing data
    def __init__(self): # initializing the class
        self.response_id = 0 # setting the response id
        self.number_of_messages = 0 # setting the number of messages
        self.message_data = [] # setting the message data


class ResponseData:  # class used for storing data


    def __init__(self): # initializing the class
        self.segment_id = 0 # stores the segment id

    gripper_command = []  # stores gripper commands
    trajectory_data = []  # stores trajectory waypoints in 4 segments

    def init_trajectory_data(self): # function to initialize the trajectory
        # empty the variable for storing trajectory
        self.trajectory_data = [] # empty the variable for storing trajectory
        self.trajectory_data.append(np.empty((0, 6), dtype=float))
        self.segment_id = 0 # reset the segment id
        self.gripper_command = [] # reset the gripper command

    def add_waypoint(self, slice_index, row): # function to add waypoint
        self.trajectory_data[slice_index] = np.vstack([self.trajectory_data[slice_index], row])

    def add_segment(self): #    function to add segment
        self.trajectory_data.append(np.empty((0, 6), dtype=float)) # add empty segment


class RobotRequestResponseCommunication: # class used for storing data

    response_data = ResponseData()  # create object for storing data

    def __init__(self): # initializing the class
        self.active_request = 0  # variable to check, if old request has finished and new one can be called
        self.client = None # variable to store client
        self.message = None # variable to store message
        self.print_messages = True # True -> prints messages , False -> doesnt print messages

    def connect_to_server(self, CONTROLLER_IP, PORT): # function to connect to server
        self.client = socket.socket() # create socket
        self.client.connect((str(CONTROLLER_IP), PORT)) # connect to server
        msg = build_hello_msg() # build hello message
        self.client.send(msg) # send hello message

    def close_connection(self): # function to close connection
        self.client.close() # close connection

    # -------------------------------------------------------------------
    #                      BIN PICKING REQUESTS
    # -------------------------------------------------------------------
    def pho_request_init(self, vs_id, start, end):
        payload = [vs_id, 0, 0, 0]  # payload - vision system ID
        payload = payload + floatArray2bytes(start)  # payload - start
        payload = payload + floatArray2bytes(end)  # payload - end
        self.pho_send_request(PHO_INIT_REQUEST, payload)
        self.pho_receive_response(PHO_INIT_REQUEST)

    def pho_request_bps_scan(self, vs_id):
        payload = [vs_id, 0, 0, 0]  # payload - vision system ID
        self.pho_send_request(PHO_SCAN_BPS_REQUEST, payload)

    def pho_bps_wait_for_scan(self):
        self.pho_receive_response(PHO_SCAN_BPS_REQUEST)
        self.active_request = 0  # request finished - response from request received

    def pho_request_trajectory(self, vs_id):
        payload = [vs_id, 0, 0, 0]  # payload - vision system ID
        self.pho_send_request(PHO_TRAJECTORY_REQUEST, payload)
        self.pho_receive_response(PHO_TRAJECTORY_REQUEST)

    def pho_request_pick_failed(self, vs_id):
        payload = [vs_id, 0, 0, 0]  # payload - vision system ID
        self.pho_send_request(PHO_PICK_FAILED_REQUEST, payload)
        self.pho_receive_response(PHO_PICK_FAILED_REQUEST)

    def pho_request_get_object(self, vs_id):
        payload = [vs_id, 0, 0, 0]  # payload - vision system ID
        self.pho_send_request(PHO_GET_OBJECT_BPS_REQUEST, payload)
        self.pho_receive_response(PHO_GET_OBJECT_BPS_REQUEST)

    def pho_request_change_scene_status(self, scene_status_id):
        payload = [scene_status_id, 0, 0, 0]  # payload - status scene ID
        self.pho_send_request(PHO_CHANGE_SCENE_STATE_REQUEST, payload)
        self.pho_receive_response(PHO_CHANGE_SCENE_STATE_REQUEST)

    def pho_request_bsp_get_vision_system_status(self, vs_id):
        payload = [vs_id, 0, 0, 0]  # payload - vision system id
        self.pho_send_request(PHO_GET_VISION_SYSTEM_BPS_REQUEST, payload)
        self.pho_receive_response(PHO_GET_VISION_SYSTEM_BPS_REQUEST)

    # -------------------------------------------------------------------
    #                      LOCATOR REQUESTS
    # -------------------------------------------------------------------

    # parameter tool_pose used only in Hand-eye
    def pho_request_ls_scan(self, vs_id, tool_pose=None):
        if tool_pose is None:
            payload = [vs_id, 0, 0, 0]  # payload - vision system id
            self.pho_send_request(PHO_SCAN_LS_REQUEST, payload)
        else:
            assert len(tool_pose) == 7, 'Wrong tool_pose size'
            payload = [vs_id, 0, 0, 0]  # payload - vision system id
            payload = payload + floatArray2bytes(tool_pose)  # payload - start
            self.pho_send_request(PHO_SCAN_LS_REQUEST, payload)

    def pho_ls_wait_for_scan(self):
        self.pho_receive_response(PHO_SCAN_LS_REQUEST)
        self.active_request = 0  # request finished - response from request received

    def pho_request_get_objects(self, vs_id, number_of_objects):
        payload = [vs_id, 0, 0, 0]  # payload - vision system id
        payload = payload + [number_of_objects, 0, 0, 0]  # payload - number of objects
        self.pho_send_request(PHO_GET_OBJECT_LS_REQUEST, payload)
        self.pho_receive_response(PHO_GET_OBJECT_LS_REQUEST)

    def pho_request_ls_get_vision_system_status(self, vs_id):
        payload = [vs_id, 0, 0, 0]  # payload - vision system id
        self.pho_send_request(PHO_GET_VISION_SYSTEM_LS_REQUEST, payload)
        self.pho_receive_response(PHO_GET_VISION_SYSTEM_LS_REQUEST)

    def move_to_position(self, joint_angles, tolerance=0.01, timeout=30):
        try:
            start_time = time.time()
            while True:
                # Replace 'pho_get_current_joint_angles' with the actual method to retrieve the robot's joint angles
                current_joint_angles = self.robot.pho_get_current_joint_angles()  # Placeholder method
                distance = sum((current - target) ** 2 for current, target in zip(current_joint_angles, joint_angles)) ** 0.5
                
                if distance <= tolerance:
                    logging.info(f"Robot reached target joint angles: {current_joint_angles}")
                    break
                
                if time.time() - start_time > timeout:
                    raise TimeoutError("Robot did not reach the target joint angles in time.")
                
                time.sleep(0.5)
        
        except AttributeError:
            logging.error("The method 'pho_get_current_joint_angles' does not exist in CommunicationLibrary.")
        except Exception as e:
            logging.error(f"An error occurred while moving the robot to joint position: {e}")

    # -------------------------------------------------------------------
    #                      CALIBRATION REQUESTS
    # -------------------------------------------------------------------
    def pho_request_add_calibration_point(self, tool_pose):
        payload = floatArray2bytes(tool_pose)  # payload - start
        self.pho_send_request(PHO_ADD_CAL_POINT_REQUEST, payload)
        self.pho_receive_response(PHO_ADD_CAL_POINT_REQUEST)

    def pho_request_start_automatic_calibration(self, sol_id, vs_id):
        payload = [sol_id, 0, 0, 0]  # payload - solution id
        payload = payload + [vs_id, 0, 0, 0]  # payload - vision system id
        self.pho_send_request(PHO_START_AUTO_CAL_REQUEST, payload)
        self.pho_receive_response(PHO_START_AUTO_CAL_REQUEST)

    def pho_request_save_automatic_calibration(self):
        self.pho_send_request(PHO_SAVE_AUTO_CAL_REQUEST)
        self.pho_receive_response(PHO_SAVE_AUTO_CAL_REQUEST)

    def pho_request_stop_automatic_calibration(self):
        self.pho_send_request(PHO_STOP_AUTO_CAL_REQUEST)
        self.pho_receive_response(PHO_STOP_AUTO_CAL_REQUEST)

    # -------------------------------------------------------------------
    #                      SOLUTION REQUESTS
    # -------------------------------------------------------------------
    def pho_request_change_solution(self, sol_id):
        payload = [sol_id, 0, 0, 0]  # payload - vision system id
        self.pho_send_request(PHO_CHANGE_SOLUTION_REQUEST, payload)
        self.pho_receive_response(PHO_CHANGE_SOLUTION_REQUEST)

    def pho_request_start_solution(self, sol_id):
        payload = [sol_id, 0, 0, 0]  # payload - vision system id
        self.pho_send_request(PHO_START_SOLUTION_REQUEST, payload)
        self.pho_receive_response(PHO_START_SOLUTION_REQUEST)

    def pho_request_stop_solution(self):
        self.pho_send_request(PHO_STOP_SOLUTION_REQUEST)
        self.pho_receive_response(PHO_STOP_SOLUTION_REQUEST)

    def pho_request_get_running_solution(self):
        self.pho_send_request(PHO_GET_RUNNING_SOLUTION_REQUEST)
        self.pho_receive_response(PHO_GET_RUNNING_SOLUTION_REQUEST)

    def pho_request_get_available_solution(self):
        self.pho_send_request(PHO_GET_AVAILABLE_SOLUTION_REQUEST)
        self.pho_receive_response(PHO_GET_AVAILABLE_SOLUTION_REQUEST)

    # -------------------------------------------------------------------
    #                     REQUEST RELATED FUNCTIONS
    # -------------------------------------------------------------------

    def pho_send_request(self, request_id, payload=None):
        assert self.active_request == 0, "Request " + request_name[self.active_request] + " not finished"
        self.active_request = request_id
        msg = PHO_HEADER  # header - PHO
        if payload is not None:
            msg = msg + [int(len(payload) / PACKET_SIZE), 0, 0, 0]  # header - payload size
            msg = msg + [request_id, 0, 0, 0]  # header - request ID
            msg = msg + payload  # payload
        else:
            msg = msg + [0, 0, 0, 0]  # header - payload size
            msg = msg + [request_id, 0, 0, 0]  # header - request ID

        self.client.send(bytearray(msg))

    def pho_receive_response(self, required_id):
        # receive header
        received_header = self.client.recv(HEADER_SIZE)
        request_id = int.from_bytes(received_header[0:3], "little")
        number_of_messages = int.from_bytes(received_header[4:7], "little")
        assert len(received_header) == HEADER_SIZE, 'Wrong header size'
        header = ResponseHeader(request_id, number_of_messages)
        assert header.request_id == required_id, 'Wrong request id received'

        if request_id == PHO_TRAJECTORY_REQUEST: self.response_data.init_trajectory_data() # empty variable for receiving new trajectory

        for message_count in range(header.sub_headers):
            received_subheader = self.client.recv(SUBHEADER_SIZE)
            operation_type = int.from_bytes(received_subheader[0:3], "little")
            operation_number = int.from_bytes(received_subheader[4:7], "little")
            data_size = int.from_bytes(received_subheader[8:11], "little")
            assert len(received_subheader) == SUBHEADER_SIZE, 'Wrong subheader size'

            if operation_type == OperationType.PHO_TRAJECTORY_CNT or operation_type == OperationType.PHO_TRAJECTORY_FINE:
                if self.response_data.segment_id >= len(self.response_data.trajectory_data):  self.response_data.add_segment()
                waypoints = ()
                waypoint_size = 2 * PACKET_SIZE + 6 * PACKET_SIZE
                for iterator in range(data_size):
                    data = self.client.recv(waypoint_size)
                    waypoint_id = struct.unpack('<i', data[0:4])[0]
                    waypoint = struct.unpack('<6f', data[4:28])
                    check_sum = struct.unpack('<f', data[28:32])[0]
                    joint_sum = sum(waypoint)
                    assert abs(joint_sum - check_sum) < 0.01, "Wrong joints sum"
                    waypoints = waypoints + waypoint
                    self.response_data.add_waypoint(self.response_data.segment_id, waypoint)  # add waypoint to the actual segment of trajectory
                self.response_data.segment_id += 1  # increment to switch to another segment of trajectory
                self.message = waypoints
                self.print_message(operation_type)
            elif operation_type == OperationType.PHO_GRIPPER:
                data_size = data_size * 4
                data = self.client.recv(data_size)
                self.response_data.gripper_command.append(int(data[0])) # store gripper command
                self.message = data
                self.print_message(operation_type)
            elif operation_type == OperationType.PHO_ERROR:
                data_size = data_size * 4
                data = self.client.recv(data_size)
                error_code = int.from_bytes(data[0:3], "little")
                self.message = error_code
                self.print_message(operation_type)
            elif operation_type == OperationType.PHO_INFO:
                data = self.client.recv(data_size * PACKET_SIZE)
                self.message = data
                self.print_message(operation_type)
            elif operation_type == OperationType.PHO_OBJECT_POSE:
                data = self.client.recv(OBJECT_POSE_SIZE)
                object_pose = struct.unpack('<7f', data[0:28])
                self.message = object_pose
                a = self.print_message(operation_type)
                print(a)
                ServoX(robot=r).servo_x(a)
            else:
                assert False, "Unexpected operation type"

        self.active_request = 0  # request finished - response from request received

    def print_message(self, operation_type):
        if self.print_messages is not True:
            return

        if operation_type == OperationType.PHO_TRAJECTORY_CNT or operation_type == OperationType.PHO_TRAJECTORY_FINE:
            waypoints_size = int((len(self.message) + 1) / 6)
            for x in range(waypoints_size):
                print('\033[94m' + "ROBOT: " + '\033[0m' + "[" + str(round(self.message[x * 6 + 0], 2)) + "," + str(
                    round(self.message[x * 6 + 1], 2)) + "," + str(round(self.message[x * 6 + 2], 2)) + "," + str(
                    round(self.message[x * 6 + 3], 2)) + "," + str(
                    round(self.message[x * 6 + 4], 2)) + "," + str(round(self.message[x * 6 + 5], 2)) + "]")
        elif operation_type == OperationType.PHO_GRIPPER:
            print('\033[94m' + "ROBOT GRIPPER: " + '\033[0m' + "[" + str(self.message[0]) + "]")
        elif operation_type == OperationType.PHO_ERROR:
            print('\033[94m' + "ERROR CODE: " + '\033[0m' + "[" + str(self.message) + "]")
        elif operation_type == OperationType.PHO_INFO:
            data_size = int((len(self.message) + 1) / 4)
            for iterator in range(data_size):
                assert len(self.message) == data_size * PACKET_SIZE
                info = int.from_bytes(self.message[0 + iterator * PACKET_SIZE:3 + iterator * PACKET_SIZE], "little")
                print('\033[94m' + "INFO: " + '\033[0m' + "[" + str(info) + "]")
        elif operation_type == OperationType.PHO_OBJECT_POSE:
            print('\033[94m' + "OBJECT: " + '\033[0m' + "[" + str(round(self.message[0], 3)) + "," + str(
                round(self.message[1], 3)) + "," + str(round(self.message[2], 3)) + "," + str(
                round(self.message[3], 3)) + "," + str(round(self.message[4], 3)) + "," + str(
                round(self.message[5], 3)) + "," + str(round(self.message[6], 3)) + "]")
            #print(type(self.message))
            #print(list(self.message))
        return self.message

# -------------------------------------------------------------------
#                     OTHER FUNCTIONS
# -------------------------------------------------------------------

def floatArray2bytes(array): # function to convert float array to bytes
    msg = [] # creating the message
    for value in array: # iterating through the array
        msg = msg + list(struct.pack('<f', value)) # converting to bytes
    return msg # returning the message


def build_hello_msg(): # function to build the hello message
    return bytearray(BRAND_IDENTIFICATION.encode('utf-8')) # returning the message


def build_state_server_hello_msg(): # function to build the state server hello message
    return bytearray(BRAND_IDENTIFICATION_SERVER.encode('utf-8')) # returning the message


# -------------------------------------------------------------------
#                      STATE SERVER FUNCTIONS
# -------------------------------------------------------------------

class RobotStateCommunication: # class for state server
    def __init__(self): # initializing the class
        self.client = None # client
        self.server = None # server

    def create_server(self, ROBOT_CONTROLLER_IP, PORT): # defining the server
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # create socket
        self.server.bind((ROBOT_CONTROLLER_IP, PORT)) # bind
        # Listen for incoming connections
        self.server.listen(1) # listen
        print('Server is running, waiting for client...') # print the message

    def wait_for_client(self): # waiting for client
        self.client, client_address = self.server.accept() # accept connection
        print('Connection established...') # print the message
        # Send hello string
        self.client.send(build_state_server_hello_msg()) # send the message

    def close_connection(self): # closing the connection
        self.server.close() # close the connection

    def send_joint_state(self): # sending the joint state
        msg = deepcopy(PHO_HEADER) # creating the message
        msg = msg + [6, 0, 0, 0]  # Data size # Data size
        msg = msg + [JOINT_STATE_TYPE, 0, 0, 0]  # Type 
        msg = msg + floatArray2bytes(get_joint_state(init_joint_state)) # sending the joint state
        self.client.send(bytearray(msg)) # sending the message

    def send_tool_pose(self): # sending the tool pose
        msg = deepcopy(PHO_HEADER) # creating the message
        msg = msg + [7, 0, 0, 0]  # Data size
        msg = msg + [TOOL_POSE_TYPE, 0, 0, 0]  # Type
        msg = msg + floatArray2bytes(get_tool_pose(base_quat)) # sending the tool pose
        self.client.send(bytearray(msg)) # sending the message

