class VisionSystem:
    def __init__(self, camera_interface):
        self.camera = camera_interface

    def get_objects_from_camera(self):
        """
        Simulate fetching object data from the 3D vision camera.
        In a real scenario, this would interface with the camera's SDK/API.
        """
        # Simulated data from the camera
        return [
            {"id": 1, "name": "Pipe", "position": [100, 200, 300], "orientation": [0, 0, 0, 1]},
            {"id": 2, "name": "Trapezoid", "position": [150, 250, 350], "orientation": [0, 0, 0, 1]}
        ]

    def select_objects_by_id(self, ids):
        """
        Select objects from the camera data using their ID numbers.

        :param ids: List of object IDs to select.
        :return: List of selected objects.
        """
        objects = self.get_objects_from_camera()
        selected_objects = [obj for obj in objects if obj["id"] in ids]
        return selected_objects

# Example usage
if __name__ == "__main__":
    vision_system = VisionSystem(camera_interface=None)  # Replace with actual camera interface if available
    selected_objects = vision_system.select_objects_by_id([1, 2])

    for obj in selected_objects:
        print(f"Selected Object: {obj['name']} (ID: {obj['id']})")
        print(f"  Position: {obj['position']}")
        print(f"  Orientation: {obj['orientation']}\n")
