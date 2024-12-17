# import requests

# pose_translation = []
# '''
# Write code to obtain pose from camera

# while (5 sec)
#     trigger camera
#     response = getresponse
#     pose_translation = getPose(response)
# '''

# pose_rotation = [] #check other codes to understand how orientation is found out

# pose = pose_translation + pose_rotation


# #joint angles = r.ik_fk("ik", pose )
# #r.move_joint()
    

# #################### PHOTONEO CAMERA ##########################################################

# from neurapy.camera.camera import Camera
# import requests
# from scipy.spatial.transform import Rotation
# import time

# class Photoneo(Camera):

#     def create_camera_connection(self, address: str, timeout: int) -> None:
#         try:
#             requests.get(address, timeout=timeout)
#         except Exception as e:
#             print("Couldn't connect with the socket-server: %s\n terminating program" % e)
#             raise e

    
#     def set_camera_command(self, address: str, timeout: int)  -> None:
#         return requests.get(address, timeout=timeout)


#     #This function can change based on how the output from camera is set in camera calibration
#     def extract_pose_from_camera(self, cameraOut: bytes) -> list:
#         # string = cameraOut.text
#         # pattern = r'\d+\.\d+,\d+\.\d+,\d+\.\d+'
#         # numbers = re.findall(pattern, string)
#         # numbers = [float(value) / 1000 for value in numbers]
#         numbers = [float(i) for i in cameraOut.text.replace('rgRES 0','').split(',')]
#         return [numbers[0]/1000, numbers[1]/1000, numbers[2]]
        
    
#     def generate_pose(self, camera_pose):
#         theta = camera_pose[2]
#         rot = Rotation.from_euler('z', theta, degrees=False)
#         euler = rot.as_euler(degrees=False, seq='XYZ')
#         pose =  [camera_pose[0], camera_pose[1], 0.200, *euler] 

#         if pose[3] + 3.1416 > 3.1416:
#             pose[3] = abs(pose[3] - 3.1416)
#         else:
#             pose[3] = abs(pose[3] + 3.1416)
#         return pose

#     def get_pose(self) -> list: 
#         try:
#             self.create_camera_connection('http://192.168.2.20/CmdChannel?TRIG', 4)
#             response = self.set_camera_command('http://192.168.2.20/CmdChannel?gRES', 4)
#             position_from_camera = self.extract_pose_from_camera(response)
            

#             # # retry 5 times if the item is not identified by the camera
#             # count = 0
#             # while all([v == 0.0 for v in position_from_camera]):
#             #     if count == 5:
#             #         print("Couldn't find part")
#             #         raise Exception("Couldn't find part")
#             #     print("position_from_camera", position_from_camera)
#             #     time.sleep(0.3)
#             #     print("retrying %s time" % str(count+1))
#             #     self.create_camera_connection('http://192.168.2.20/CmdChannel?TRIG', 4)
#             #     response = self.set_camera_command('http://192.168.2.20/CmdChannel?gRES', 4)
#             #     position_from_camera = self.extract_pose_from_camera(response)
#             #     count += 1

#             pose_cmd = self.generate_pose(position_from_camera)
#             target_cmd = pose_cmd

#             return target_cmd
#         except Exception as e:
#             print("Exception Sick: ", e)
#             raise e

############################## DELTA MV ##################################################################

import math
import socket
import sys
import time

from scipy.spatial.transform import Rotation

from neurapy.camera.camera import Camera


class DeltaDMV(Camera):
    def __init__(self):
        self.sock = None

    def connect(self, ip_address: str, port: int) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = (ip_address, port)
        try:
            self.sock.connect(server_address)
        except socket.error as msg:
            print("Couldn't connect with the socket-server: %s\n terminating program" % msg)
            sys.exit(1)

    def disconnect(self) -> None:
        self.sock.close()

    def trigger(self) -> bytes:
        self.sock.sendall("T1\r\n".encode())
        time.sleep(0.3)   

    def generate_pose(self, camera_pose):
        theta = math.radians(camera_pose[2])
        rot = Rotation.from_euler('z', theta, degrees=False)
        euler = rot.as_euler(degrees=False, seq='XYZ')
        pose =  [camera_pose[0]/1000, camera_pose[1]/1000, 0.12, -3.13, 0.0, -1.57] 

        # if pose[3] + 3.1416 > 3.1416:
        #     pose[3] = abs(pose[3] - 3.1416)
        # else:
        #     pose[3] = abs(pose[3] + 3.1416)
        return pose


    def get_pose(self) -> list:
        self.connect("192.168.2.33", 502)
        self.trigger()

        try:
            self.sock.sendall("DQ\r\n".encode())
            buffer = b''
            while b'\r' not in buffer:
                data = self.sock.recv(1024)
                if not data: # socket closed
                    break
                buffer += data
            buffer = buffer[2:]
            line,sep,buffer = buffer.partition(b'\r')
            values = line.decode().split(',')
            values = [float(value) for value in values]

            pose = self.generate_pose(values)
        except Exception as e:
            print("Exception DMV error formed: ", e)
            # self.emit_message_to_gui(str(e), "Error")
            # raise e
        
        self.disconnect()

        return pose


if __name__ == "__main__":
    camera = DeltaDMV()
    camera.connect("192.168.2.33", 502)
    camera.trigger()
    print("final",camera.get_pose())
    camera.disconnect()


