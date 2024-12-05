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
