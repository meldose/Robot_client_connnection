from palletizing.camera import Camera
import socket
import sys
import math
import time

class Keyence(Camera):
    
    def create_camera_connection(self, ip_address: str, port: int) -> None:
        server_address = (ip_address, port)
        try:
            self.sock.connect(server_address)
        except socket.error as msg:
            print("Couldn't connect with the socket-server: %s\n terminating program" % msg)
            sys.exit(1)

    
    
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
            self.create_camera_connection("192.168.3.10", 8500)
            pose_cmd = self.set_camera_command("T1;")
            target_cmd = self.extract_pose_from_camera(pose_cmd)

            #retry 5 times if the item is not identified by the camera
            count = 1
            while all([v == 0.0 for v in target_cmd]):
                if count == 5:
                    message = "Couldn't find part"
                    self.emit_message_to_gui(message, "Error")
                time.sleep(1)
                print("retrying %s time" % str(count))
                pose_cmd = self.set_camera_command("T1;")
                target_cmd = self.extract_pose_from_camera(pose_cmd)
                count += 1

            self.disconnect_camera_connection()
        except Exception as e:
            print("Exception: ", e)
            self.emit_message_to_gui(str(e), "Error")
            raise e

        return target_cmd
