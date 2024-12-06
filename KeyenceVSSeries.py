##from neurapy.robot import Robot # type: ignore
import socket
import math
import time
from neurapy.camera.camera import Camera

#r=Robot()

class KeyenceVSSeries(Camera):
    def __init__(self):
        self.ip_address = '192.168.2.7' #Check the IP address on which camera is configured
        self.port = 8500 # check the PORT
        self.camera_cmd = "TRG;"
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.create_camera_connection()

    def create_camera_connection(self):
        # Connect the socket to the port where the server is listening
        server_address = (self.ip_address, self.port)
        try:
            print('connecting to %s port %s' % server_address)
            self.sock.connect(server_address)
            print('connected to %s port %s' % server_address)
        except socket.error as msg:
            print("Couldn't connect with the socket-server: %s\n terminating program" % msg)
            raise Exception(f"Warning [Keyence VS] Connection issue: {msg}")

    def close_camera_connection(self):
        self.sock.close()


    def extract_pose_from_camera(self, camera_res):
        # Convert to correct format
        target_cmd = [float(value) for value in camera_res]
        target_cmd[3:] = [math.radians(value) for value in target_cmd[3:]] #<--------------- 3:
        target_cmd[:3] = [value / 1000 for value in target_cmd[:3]]
        return target_cmd


    def set_camera_command(self, command: str) -> bytes:
        self.sock.sendall(command.encode('utf-8'))
        snd_cmd = self.sock.recv(16)
        dummy_cmd = self.sock.recv(16)
        pose_cmd = self.sock.recv(1024)
        return pose_cmd


    def get_pose(self):
        try:
            camera_res = self.set_camera_command(self.camera_cmd)
            camera_res_decoded = pose_cmd.decode('utf-8').replace(';',' ').split(',')

            #retry 5 times if the item is not identified by the camera
            count = 1
            while len(camera_res_decoded) != 6:
                if count == 5:
                    print("Couldn't find part")
                    sys.exit()
                time.sleep(1)
                print(f"Trying camera {count} times")
                camera_res = self.set_camera_command(self.camera_cmd)
                camera_res_decoded = pose_cmd.decode('utf-8').replace(';',' ').split(',')

            target_cmd = self.extract_pose_from_camera(camera_res_decoded)

        except Exception as e:
            print("Exception: ", e)
            self.disconnect_camera_connection()
            raise Exception(f"Warning [Keyence VS] Error {e}")

        return target_cmd

if __name__ == "__main__":

    cam = Keyence_VS()

    for i in range(5):
        pose = cam.get_pose()
