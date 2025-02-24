# -------------------------------------------------------------------
#                      IMPORTS
# -------------------------------------------------------------------

import socket # importing socket
from copy import deepcopy # importing copy
import struct # importing struct
import math # importing math
import numpy as np # importing numpy
from StateServer import get_joint_state, get_tool_pose, init_joint_state, base_quat
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

# -------------------------------------------------------------------
#                      SERVO_J
# -------------------------------------------------------------------

# import copy # importing copy moudule

# class ServoJ:  # defining servoJ
    
#     def __init__(self, robot):  # initializing the robot
#         self.robot = robot  # setting the robot

#     def servo_j(self, message): # defining the servo_j function
#         message = [x / 1000 for x in message]  # Scale values
        
#         x = message[0] # Scale values
#         y = message[1] # Scale values
#         z = message[2] # Scale values
#         a = message[3] # Scale values
#         b = message[4] # Scale values
#         c = message[5] # Scale values
#         d = message[6] # Scale values

#         print(message)# printing the message
        
#         new_message = [x, y,z,d, a, b, c] # added new order for quaternion values

#         print(new_message)# printing the new ordered message

#         # Activate servo interface
#         r.activate_servo_interface('position')
#         dof = 6  # Degrees of freedom
#         otg = Ruckig(dof, 0.001)  # Online trajectory generator

#         # Input/Output parameters
#         inp = InputParameter(dof) #setting the input parameter
#         out = OutputParameter(dof) #setting the output parameter

#         # Current state
#         inp.current_position = r.get_current_joint_angles() # getting the current joint angles
#         inp.current_velocity = [0.0] * dof # setting the current velocity as zero
#         inp.current_acceleration = [0.0] * dof # setting the current acceleration as zero

#         target_joint_angles = r.ik_fk("ik",target_pose=new_message, # conversion of target pose
#         current_joint=[0.4129184862608269,-0.04035147853479624,-1.6033459562606136,-0.07107998043766754,-1.5406373722142601,0.910522489241973])
#         print("Target Joint Angles:", target_joint_angles) # print the target joint angles

#         inp.target_position = target_joint_angles # setting the target position
#         target = copy.deepcopy(inp.current_position) # copying the current position of the robot 
#         # inp.target_position = [0.31764351712572647, -1.5097579644424788, -1.115881588855747, 1.8344006543935802, -2.4782356003958528, -0.6248432487824395] # setting the target position
#         inp.target_position = [new_message[0], new_message[1], new_message[2], target[3], target[4], target[5], target[6]] # passigng the values by fixing the [X,Y,z and fixign the d,a,b,c]
#         inp.target_acceleration = [0.0] * dof # setting the target acceleration as zero
#         inp.max_velocity = [0.5] * dof #    defining the maximum velocity
#         inp.max_acceleration = [3.0] * dof # defining the maximum acceleration
#         inp.max_jerk = [10.0] * dof # defining the maximum jerk

#         res = Result.Working # setting the res variable as Result.Working

#         while res == Result.Working: # while the result is working
#             res = otg.update(inp, out) # updating the input and output
#             error_code = r.servo_j(out.new_position, out.new_velocity, out.new_acceleration) # passing the error code variable with having servo_j function having position, velocity and acceleration
#             scaling_factor = r.get_servo_trajectory_scaling_factor() # getting the servo trajectory scaling factors
#             out.pass_to_input(inp) # passing the output to the input
#             time.sleep(0.001) # setting the time sleep to 0.001 seconds

#         r.deactivate_servo_interface() # deactivating the servo interface
#         # r.stop() # stopped the robot
#         r.move_joint("P19") # moving to P19
#         r.gripper("off")
#         r.move_joint("P20") # moving to P20
#         r.gripper("on") # setting gripper on
#         r.move_joint("P16") # moving to P16

#     # ServoX(robot=r).servo_x()
#     r.set_mode("Automatic") # setting the mode to automatic
#     r.gripper("on") # setting the gripper on
#     r.move_joint("P16") # moving to P16
#     r.gripper("off") # setting the gripper off

# -------------------------------------------------------------------
#                      SERVO_X (WORKING)
# -------------------------------------------------------------------

import copy # importing copy module

r=Robot()

class ServoX: # defining servoX

    def __init__(self,robot): # initializing the robot
        self.robot = robot # setting the robot

    
    def servo_x(self,message,*args,**kwargs): # defining servoX

        r=Robot()
 
        message = [x/1000 for x in message] # converting the values to mm
        
        x = message[0]  # setting the values
        y = message[1]  # setting the values
        z = message[2]  # setting the values
        a = message[3]  # setting the values
        b = message[4]  # setting the values
        c = message[5]  # setting the values
        d = message[6]  # setting the values
        
        print(message) # printing the message
        
        new_message = [x,y,z,d,a,b,c] # added new order for quaternion values
        print(new_message) # printing the new ordered message


        # r = self.robot #setting the robot

        # #Switch to external servo mode
        r.activate_servo_interface('position') # activating the servo interface

        cart_pose_length = 7 #X,Y,Z,qw,qx,qy,qz  

        otg = Ruckig(cart_pose_length, 0.001)  # control cycle
        inp = InputParameter(cart_pose_length) # setting the inputparameter with cart pose length
        out = OutputParameter(cart_pose_length) # setting the outputparmeter with cart pose length

        inp.current_position = r.get_current_cartesian_pose() # getting the current cartesian poses
        inp.current_velocity = [0.]*cart_pose_length # mutliplying the initial velocity with cart pose lenght 
        inp.current_acceleration = [0.]*cart_pose_length # mutliplying the current acceleration with cart pose length

        # target = copy.deepcopy(inp.current_position) # copying the current position of the robot 
        # inp.target_velocity = [0.]*cart_pose_length # defning the target velocity
        # inp.target_acceleration = [0.]*cart_pose_length # definng the target acceleration

        target = copy.deepcopy(inp.current_position) # copying the current position of the robot 
        inp.target_position = [new_message[0], new_message[1], new_message[2], target[3], target[4], target[5], target[6]] # providing the target position
        # inp.target_position = new_message
        inp.target_velocity = [0.]*cart_pose_length # defning the target velocity
        inp.target_acceleration = [0.]*cart_pose_length # definng the target acceleration

        inp.max_velocity = [0.8]*cart_pose_length # setting the maximum velocity with 0.5 times the cart pose lenght 
        inp.max_acceleration = [3]*cart_pose_length #se tting the max acceleration with 3 times the cart pose length
        inp.max_jerk = [10.]*cart_pose_length # setting the jerk values

        servox_proportional_gain = 25 # setting the servox propotional gain as 25

        velocity = [0.] * 6 #Since ruckig does not provide rotational velocity if quaternion is input, we can send 0 rotational feedforward velocity
        acceleration = [0.] * 6 #Since ruckig does not provide rotational acceleration if quaternion is input, we can send 0 rotational feedforward acceleration
        
        res=Result.Working # setting the result

        while res == Result.Working: # while the result is working
            
            error_code = 0 # setting the error code

            res = otg.update(inp, out) # updating the input and output

            position = out.new_position # getting the new position

            # for i in range(0,3): # Updating target translation velocity and accelerations
                # velocity[i] = out.new_velocity[i]
                # acceleration[i] = out.new_acceleration[i]

            zeros = [0.] * 6 #setting the zeros

            error_code = r.servo_x(position, zeros, zeros, servox_proportional_gain) # passing the error code variable with having servo_j function having position, velocity and acceleration
            scaling_factor = r.get_servo_trajectory_scaling_factor() # getting the servo trajectory scaling factors
            out.pass_to_input(inp)
            time.sleep(0.001) # setting time 
            
        r.deactivate_servo_interface() # deactivating the servo interface
        r.gripper("off") # setting gripper close position
        r.move_joint("P19") # moving to p19
        r.move_joint("P20") # moving to P20
        r.gripper("on") # setting gripper on
        r.move_joint("P28") # moving to P28

        # r.stop() # stopping the robot
    
    r.set_mode("Automatic") # setting the mode to automatic
    r.gripper("on") # setting the gripper on
    r.move_joint("P28") # moving to P28

# -------------------------------------------------------------------
#                      MOVE_LINEAR (WORKING)
# -------------------------------------------------------------------

# import copy # importing copy module

# class ServoX: # defining servoX

#     def __init__(self,robot):
#         self.robot = robot # setting the robot

#     def movelinear_online(self,message,*args,**kwargs):# defining movelinear_online functionq

#         message = [x/1000 for x in message] # converting the values to mm
        
#         x = message[0]  # setting the values
#         y = message[1]  # setting the values
#         z = message[2]  # setting the values
#         a = message[3]  # setting the values
#         b = message[4]  # setting the values
#         c = message[5]  # setting the values
#         d = message[6]  # setting the values
        
#         print(message) # printing the message
        
#         new_message = [x,y,z,d,a,b,c] # added new order for quaternion values
#         print(new_message) # printing the new ordered message
        
#         #Switch to external servo mode
#         r.activate_servo_interface('position') # activating the servo interface

#         cart_pose_length = 7 # X,Y,Z,qw,qx,qy,qz
#         velocity = [0.2]*6 # setting the velocity 
#         acceleration = [2.0]*6 # setting the acceleration
#         target = copy.deepcopy(r.get_current_cartesian_pose()) # getting the current cartesian poses
#         time.sleep(1.0) # setting the time

#         # target=new_message # setting the target position 
#         target = [new_message[0], new_message[1], new_message[2], target[3], target[4], target[5], target[6]]
#         error_code = r.movelinear_online(target, velocity, acceleration) # moving the robot
#         r.gripper("on")

#         time.sleep(1.0) # setting the time

#         r.deactivate_servo_interface() # deactivating the servo interface
#         r.gripper("off") # setting gripper close position
#         r.move_joint("P19") # moving to P19
#         r.move_joint("P20") # moving to P20
#         r.gripper("on") # setting gripper close position
#         r.move_joint("P28") # moving to P28

#     r.set_mode("Automatic") # setting the robot to Automatic Mode
#     r.move_joint("P28") # movin robot to the position 28    
#     r.set_mode("Teach") # setting the mode to Teach mode
#     r.gripper("on") # setting the gripper 

# -------------------------------------------------------------------
#                      CLASSES
# -------------------------------------------------------------------

class ResponseHeader: # class used for storing data
    def __init__(self, request_id, sub_headers,number_of_messages): # initializing the class
        self.request_id = request_id # setting the request id
        self.sub_headers = sub_headers # setting the sub headers
        self.number_of_messages = number_of_messages # setting the number of messages

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


# -------------------------------------------------------------------
#                      ROBOT COMMUNICATION
# -------------------------------------------------------------------

class RobotRequestResponseCommunication: # class used for storing data

    response_data = ResponseData()  # create object for storing data

    def __init__(self,robot=None): # initializing the class
        self.active_request = 0  # variable to check, if old request has finished and new one can be called
        self.client = None # variable to store client
        self.message = None # variable to store message
        self.print_messages = True # True -> prints messages , False -> doesnt print messages
        self.robot=robot

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

    def pho_request_ls_scan(self, vs_id_1,tool_pose=None,payload=None): # defining an function for locator scan (for trapezoid )

        valid_ids={1:"Trapezoid",2:"Pipe"} # setting the list for trapezoid and pipe
    
        if vs_id_1 not in valid_ids : # checking if the trapezoid is there in the list or not

            raise ValueError("Invalid vs_id! Use 1 for Pipe, 2 for Trapezoid.") # if not raise the error
                  
        payload_1 = [vs_id_1, 0, 0, 0] # setting the payload for vision system 1
    
        if tool_pose is not None: # checking if the tool pose is None or not

            assert len(tool_pose) == 7, 'Wrong tool_pose size' # checking if the tool pose is 7
            payload_1 = payload_1 + floatArray2bytes(tool_pose) # setting the payload 1

        self.pho_send_request(PHO_SCAN_LS_REQUEST, payload_1) # sending the request to the camera with vision system1
 

    def pho_request_ls_scan_2(self,vs_id_2,tool_pose=None,payload=None): # setting the function for request scan 2

        valid_ids={1:"Trapezoid",2:"Pipe"} # setting the list for trapezoid and pipe
    
        if vs_id_2 not in valid_ids : # checking if the pipe is there in the list or not

            raise ValueError("Invalid vs_id! Use 1 for Pipe, 2 for Trapezoid.") # if not raise the error
                  
        payload_2 = [vs_id_2, 0, 0, 0] # setting the payload for vision system 2


        if tool_pose is not None: # checking if the tool pose is None or not

            assert len(tool_pose) == 7, 'Wrong tool_pose size' # checking if the lenght of the tool pose is 7 or not
            payload_2 = payload_2 + floatArray2bytes(tool_pose) # setting the payload_2
  
        self.pho_send_request(PHO_SCAN_LS_REQUEST, payload_2) # sending the request to the camera with vision system2

            
    # def pho_ls_wait_for_scan(self,vs_id,pay_load_1=None,pay_load_2=None):
    def pho_ls_wait_for_scan(self,vs_id_1,payload_1=None): # defining the function for scan wait
        
        try:

            if payload_1 is None: # if the payload is None then

                payload_1 = [vs_id_1, 0, 0, 0] # setting the vision system 1

            logging.info(f"Waiting for scan from Vision System {vs_id_1} ({'Trapezoid' if vs_id_1 == 1 else 'Pipe'})") # logging error

            self.pho_receive_response(PHO_SCAN_LS_REQUEST) # sending the request to the camera
            self.active_request = 0  # Request finished - response received

        except Exception as e:
            logging.error(f"Error in pho_ls_wait_for_scan: {e}") # popping up the error 
   
    
        # def pho_ls_wait_for_scan(self,vs_id,pay_load_1=None,pay_load_2=None):
    def pho_ls_wait_for_scan_2(self,vs_id_2,payload_2=None): # defining the function for the wait for the scan for the object 2
        
        try:

            if payload_2 is None: # setting the payload2
            
                payload_2 = [vs_id_2, 0, 0, 0] # setting the vision system 2

            logging.info(f"Waiting for scan from Vision System {vs_id_2} ({'Pipe' if vs_id_2 == 2 else 'Trapezoid'})") # logging error

            self.pho_receive_response(PHO_SCAN_LS_REQUEST) # sending the request to the camera
            self.active_request = 0  # Request finished - response received

        except Exception as e:
            logging.error(f"Error in pho_ls_wait_for_scan: {e}") # logging error

    def pho_request_get_objects(self, vs_id_1,number_of_objects_1): # defining the function for get objects
        
        try:
            # Validate input types
            if not all(isinstance(x, int) for x in [vs_id_1, number_of_objects_1]): # checking the element in the list of vision system 1 is integer or not
                if number_of_objects_1 <= 0 : # checking if the number of objects is less that or equal to zero
                    raise ValueError("number_of_objects must be greater than zero.") # if less raise the error
        
            payload_1 = [vs_id_1, 0, 0, 0, number_of_objects_1, 0, 0, 0] # setting the payload 1
            
            self.pho_send_request(PHO_GET_OBJECT_LS_REQUEST, payload_1) # sending the request to the camera with the vision system 1
            self.pho_receive_response(PHO_GET_OBJECT_LS_REQUEST) # getting tne reponse from the camera

        except Exception as e:
            logging.error(f"Error in pho_request_get_objects: {e}") # logging error

    def pho_request_get_objects_2(self,vs_id_2, number_of_objects_2): # defining the function for requesting the object for pipe
        
        try:
            # Validate input types
            if not all(isinstance(x, int) for x in [vs_id_2, number_of_objects_2]): # checking the element in the list of vision system 2 is integer or not
                if number_of_objects_2 <= 0 : # checking if the number of objects is less that or equal to zero
                    raise ValueError("vs_id and number_of_objects must be integers.")  # if less raise the errors
            
            payload_2 = [vs_id_2, 0, 0, 0, number_of_objects_2, 0, 0, 0] # setting the payload 2
            
            self.pho_send_request(PHO_GET_OBJECT_LS_REQUEST, payload_2) # sending the request to the camera with the vision system 2
            self.pho_receive_response(PHO_GET_OBJECT_LS_REQUEST) # recieving the response from the camera

        except Exception as e:
            logging.error(f"Error in pho_request_get_objects: {e}") # getting the logging error

    def pho_request_ls_get_vision_system_status(self, vs_id_1,payload_1=None): # defining the function for getting the vision systeme status
        
        if payload_1 is None: # setting if the payload is None 
             
            payload_1= [vs_id_1, 0, 0, 0] # setting the payload
            self.pho_send_request(PHO_GET_VISION_SYSTEM_LS_REQUEST, payload_1) # sending the request to the camera having the vision system
            self.pho_receive_response(PHO_GET_VISION_SYSTEM_LS_REQUEST) # recieving the request from the camera


    def pho_request_ls_get_vision_system_status_2(self, vs_id_2,payload_2=None): # defining the function for getting the vision systeme status
        
        if payload_2 is None: # setting the payload2 as None
             
            payload_2= [vs_id_2, 0, 0, 0] # setting the payload
            self.pho_send_request(PHO_GET_VISION_SYSTEM_LS_REQUEST, payload_2) # sending the request to the camera having the vision system
            self.pho_receive_response(PHO_GET_VISION_SYSTEM_LS_REQUEST) # recieving the request from the camera
            
    def move_to_position(self, joint_angles, tolerance=0.01, timeout=30): # defining the function for moving to the position
        try:
            start_time = time.time() # setting the start time
            while True:
                current_joint_angles = self.robot.pho_get_current_joint_angles()  # Placeholder method
                distance = sum((current - target) ** 2 for current, target in zip(current_joint_angles, joint_angles)) ** 0.5 # setting the distance

                if distance <= tolerance: # chceking if the distance is less than the tolerance
                    logging.info(f"Robot reached target joint angles: {current_joint_angles}") # give the logging error
                    break

                if time.time() - start_time > timeout: # if the start time is greater than timeout then
                    raise TimeoutError("Robot did not reach the target joint angles in time.") # raise the error

                time.sleep(0.5) # setting the time

        except AttributeError:
            logging.error("The method 'pho_get_current_joint_angles' does not exist in CommunicationLibrary.") # logging error
        except Exception as e:
            logging.error(f"An error occurred while moving the robot to joint position: {e}") # logging error
    
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

    def pho_send_request(self, request_id, payload=None): # defining the function for sending the request to the camera
        # assert self.active_request == 0, "Request " + request_name[self.active_request] + " not finished"
        self.active_request = request_id #setting the self.active_request to request_id
        msg = PHO_HEADER  # header - PHO
        if payload is not None: # chhecking if the payload is not None
            msg = msg + [int(len(payload) / PACKET_SIZE), 0, 0, 0]  # header - payload size
            msg = msg + [request_id, 0, 0, 0]  # header - request ID
            msg = msg + payload  # payload
        else:
            msg = msg + [0, 0, 0, 0]  # header - payload size
            msg = msg + [request_id, 0, 0, 0]  # header - request ID

        self.client.send(bytearray(msg))


    def pho_receive_response(self, required_id=None, response=None):
        # if message is None:
        #     message=list(self.client.recv(OBJECT_POSE_SIZE))

        # if not message:
        #     logging.error("Invalid or missing message data")
        #     return [],[]

        # if len(message) != OBJECT_POSE_SIZE:
        #     logging.error("Incomplete data received")
        #     return [], []
        
        # message = [x/1000 for x in message] # converting the values to mm
        
        # x = message[0]  # setting the values
        # y = message[1]  # setting the values
        # z = message[2]  # setting the values
        # a = message[3]  # setting the values
        # b = message[4]  # setting the values
        # c = message[5]  # setting the values
        # d = message[6]  # setting the values
        
        # print(message) # printing the message

        # new_message = [x,y,z,d,a,b,c] # added new order for quaternion values
        # logging.info(f"Processed message: {new_message}")

        # print(new_message)
    
        # position=new_message[:3]
        # orientation=new_message[3:]

        # # Simulated Response (Replace with actual data retrieval method)
        # if response is None:
        #     response = [
        #         {"vs_id": 1,"id": 1, "name": "Pipe", "position": position, "orientation": orientation},
        #         {"vs_id": 2,"id": 2, "name": "Trapezoid", "position": position, "orientation": orientation}
        #     ]
        # Receive header
        received_header = self.client.recv(HEADER_SIZE)
        if len(received_header) < HEADER_SIZE: # checking if the header is empty
            return[],[] # returning empty response
        request_id = int.from_bytes(received_header[0:3], "little") # setting the request id
        number_of_messages = int.from_bytes(received_header[4:7], "little") # setting the number of messages
        assert len(received_header) == HEADER_SIZE, 'Wrong header size' # checking if the header size is correct
        header = ResponseHeader(self,request_id, number_of_messages) # setting the header
        # assert header.request_id == request_id, "Wrong request_id recieved"
 
        # if required_id is not None:
        #     assert header.request_id == required_id, f"Expected request id {required_id}, but got {header.request_id}"

        if request_id == PHO_TRAJECTORY_REQUEST: # if the request id is PHO_TRAJECTORY_REQUEST
            self.response_data.init_trajectory_data()  # empty variable for receiving new trajectory

        for message_count in range(header.number_of_messages):  # Fix: using correct header field
            # Receive subheader
            received_subheader = self.client.recv(SUBHEADER_SIZE) # receiving the subheader
            operation_type = int.from_bytes(received_subheader[0:3], "little") # setting the operation type
            operation_number = int.from_bytes(received_subheader[4:7], "little") # setting the operation number
            data_size = int.from_bytes(received_subheader[8:11], "little") # setting the data size
            assert len(received_subheader) == SUBHEADER_SIZE, 'Wrong subheader size' #checking if the subheader size is correct

            if operation_type == OperationType.PHO_TRAJECTORY_CNT or operation_type == OperationType.PHO_TRAJECTORY_FINE: # checking if the operation type is PHO_TRAJECTORY_CNT or PHO_TRAJECTORY_FINE
                if self.response_data.segment_id >= len(self.response_data.trajectory_data): # checking if the segment id is greater than the length of the trajectory data
                    self.response_data.add_segment()
                waypoints = [] # crating empty list for waypoints
                waypoint_size = 8 * PACKET_SIZE  # 2 + 6
                for i in range(data_size): # iterating over the data size
                    data = self.client.recv(waypoint_size)
                    waypoint_id = struct.unpack('<i', data[0:4])[0]
                    waypoint = struct.unpack('<6f', data[4:28])
                    check_sum = struct.unpack('<f', data[28:32])[0]
                    joint_sum = sum(waypoint)
                    assert abs(joint_sum - check_sum) < 0.01, "Wrong joints sum"
                    waypoints.append(waypoint) # appending the waypoint
                    self.response_data.add_waypoint(self.response_data.segment_id, waypoint)
                self.response_data.segment_id += 1 # incrementing the segment id
                self.message = waypoints # setting the message
                self.print_message(operation_type) # printing the message

            elif operation_type == OperationType.PHO_GRIPPER: # checking if the operation type is PHO_GRIPPER
                data_size = data_size * 4 # setting the data size
                data = self.client.recv(data_size) # receiving the data
                self.response_data.gripper_command.append(int(data[0]))  # store gripper command
                self.message = data # setting the message
                self.print_message(operation_type) # printing the message

            elif operation_type == OperationType.PHO_ERROR: # checking if the operation type PHO.error
                data_size = data_size * 4 # setting the data size
                data = self.client.recv(data_size) # receiving the data
                error_code = int.from_bytes(data[0:3], "little") # setting the error code
                self.message = error_code # setting the message
                self.print_message(operation_type) # printing the message

            elif operation_type == OperationType.PHO_INFO: # checking if the operation type is PHO_INFO
                data = self.client.recv(data_size * PACKET_SIZE) # receiving the data
                self.message = data # setting the message
                self.print_message(operation_type) # printing the message

            elif operation_type == OperationType.PHO_OBJECT_POSE:  # checking if the operation type is PHO_OBJECT_POSE
                # Use simulated response if available, otherwise fetch real data
                if response:  # if there is a response
                    logging.info("Using simulated response for object pose.") # logging info
                    pose_to_move = [
                        obj["position"] + obj["orientation"] for obj in response 
                    ]
                else:
                    # Receiving real data (from robot's vision system)
                    data = self.client.recv(OBJECT_POSE_SIZE) # setting the data 
                    object_pose = struct.unpack('<7f', data[0:28]) # setting the object pose

                    # Extract position and orientation with correct order
                    position = list(object_pose[:3])  # First three values (x, y, z)
                    orientation = [object_pose[6], object_pose[3], object_pose[4], object_pose[5]]  # Reordering (d, a, b, c)

                    # Combine for movement
                    pose_to_move = [position + orientation] # setting the pose_to_move having the combination of position and orientaiton
                    self.message = position + orientation  # Store message for logging

                self.print_message(operation_type)  # printing the message

                # Ensure the pose is valid before attempting to move
                if pose_to_move:  # if there is a pose to move
                    for pose in pose_to_move:  # iterating over the poses
                        ServoX(robot=self.robot).servo_x(pose)  # Move to pose using movelinear_online

                self.active_request = 0  # Request finished - response from request received
        return response  # returning the response


    def print_message(self, operation_type): #defining the print_message function
        if self.print_messages is not True: # checking if the print messages is not true
            return [] # return the list values

        if operation_type == OperationType.PHO_TRAJECTORY_CNT or operation_type == OperationType.PHO_TRAJECTORY_FINE: # setting the operation type
            waypoints_size = int((len(self.message) + 1) / 6) # setting the waypoints_size
            for x in range(waypoints_size): # checking the waypointsize in a loop
                print('\033[94m' + "ROBOT: " + '\033[0m' + "[" + str(round(self.message[x * 6 + 0], 2)) + "," + str(
                    round(self.message[x * 6 + 1], 2)) + "," + str(round(self.message[x * 6 + 2], 2)) + "," + str(
                    round(self.message[x * 6 + 3], 2)) + "," + str(
                    round(self.message[x * 6 + 4], 2)) + "," + str(round(self.message[x * 6 + 5], 2)) + "]")
        elif operation_type == OperationType.PHO_GRIPPER: # setting the operaiton type to Gripper
            print('\033[94m' + "ROBOT GRIPPER: " + '\033[0m' + "[" + str(self.message[0]) + "]")
        elif operation_type == OperationType.PHO_ERROR: # setting the operationtype to Photoneo Error
            print('\033[94m' + "ERROR CODE: " + '\033[0m' + "[" + str(self.message) + "]")
        elif operation_type == OperationType.PHO_INFO: # setting the operation type photoeneo.info
            data_size = int((len(self.message) + 1) / 4) # setting the datasize having the integer 
            for iterator in range(data_size): # checkign the iterator in the range for data size
                assert len(self.message) == data_size * PACKET_SIZE # checking if the lenght of the self.message is equal to the data_size * PACKET_SIZE
                info = int.from_bytes(self.message[0 + iterator * PACKET_SIZE:3 + iterator * PACKET_SIZE], "little")
                print('\033[94m' + "INFO: " + '\033[0m' + "[" + str(info) + "]")
        elif operation_type == OperationType.PHO_OBJECT_POSE: # setting the operation type to the Photoneo object pose
                print('\033[94m' + "OBJECT: " + '\033[0m' + "[" + str(round(self.message[0], 3)) + "," + str(
                    round(self.message[1], 3)) + "," + str(round(self.message[2], 3)) + "," + str(
                    round(self.message[3], 3)) + "," + str(round(self.message[4], 3)) + "," + str(
                    round(self.message[5], 3)) + "," + str(round(self.message[6], 3)) + "]")
                
        return self.message # returning the self.messsage

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

