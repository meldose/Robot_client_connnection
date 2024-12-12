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
            print("Exception DMV: ", e)
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

