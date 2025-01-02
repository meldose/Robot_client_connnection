from neurapy.camera.camera import Camera
import socket
import sys
import math
import time

class KeyenceCVSeries(Camera):
    def __init__(self):
        self.ip_address = '192.168.2.33' #Check the IP address on which camera is configured
        self.port = 8500 # check the PORT
        self.camera_cmd = "T1;"
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.create_camera_connection()
    

    def create_camera_connection(self) -> None:
        # Connect the socket to the port where the server is listening
        server_address = (self.ip_address, self.port)
        try:
            print('connecting to %s port %s' % server_address)
            self.sock.connect(server_address)
            print('connected to %s port %s' % server_address)
        except socket.error as msg:
            print("Couldn't connect with the socket-server: %s\n terminating program" % msg)
            raise Exception(f"Warning [Keyence CV] Connection issue: {msg}")
                 
    
    def disconnect_camera_connection(self) -> None:
        self.sock.close()

    
    def extract_pose_from_camera(self, cameraOut: bytes) -> list:
        pose_values = cameraOut.decode('utf-8').split(',')
        target_cmd = [float(value) for value in pose_values]
        target_cmd[3:] = [math.radians(value) for value in target_cmd[3:]]
        target_cmd[:3] = [value / 1000 for value in target_cmd[:3]]
        return target_cmd

    
    def set_camera_command(self, command: str) -> bytes:
        self.sock.sendall(command.encode('utf-8'))
        send_cmd = self.sock.recv(16)
        pose_cmd = self.sock.recv(1024)
        return pose_cmd

    
    def get_pose(self) -> list:
        try:
            camera_res = self.set_camera_command(self.camera_cmd)
            target_cmd = self.extract_pose_from_camera(camera_res)
            
            #retry 5 times if the item is not identified by the camera
            count = 1
            while all([v == 0.0 for v in target_cmd]):
                if count == 5:
                    print("Couldn't find part")
                    sys.exit()
                time.sleep(0.1)
                print(f"Trying camera {count} times")
                camera_res = self.set_camera_command(self.camera_cmd)
                target_cmd = self.extract_pose_from_camera(camera_res)
                count += 1

        except Exception as e:
            print("Exception: ", e)
            self.disconnect_camera_connection()
            raise Exception(f"Warning [Keyence CV] Error {e}")

        return target_cmd
