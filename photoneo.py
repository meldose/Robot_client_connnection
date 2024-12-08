import requests

pose_translation = []
'''
Write code to obtain pose from camera

while (5 sec)
    trigger camera
    response = getresponse
    pose_translation = getPose(response)
'''

pose_rotation = [] #check other codes to understand how orientation is found out

pose = pose_translation + pose_rotation


#joint angles = r.ik_fk("ik", pose )
#r.move_joint()


import os

from neurapy.camera.camera import Camera

class DummyCamera(Camera):
    def __init__(self, file_path='~/pose.txt'):
        self.file_path = os.path.expanduser(file_path)

    def read_pose_from_file(self) -> list:
        try:
            with open(self.file_path, 'r') as file:
                line = file.readline().strip()
                pose_data = [float(value) for value in line.split(',')]
                return pose_data
        except Exception as e:
            print(f"Failed to read pose from file: {e}")
            raise

    def get_pose(self) -> list:
        return self.read_pose_from_file()
