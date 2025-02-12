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
        
        
#################################################################################
 
 
class VisionSystemController:
    def __init__(self, vs_id):
    self.vs_id = vs_id  # Vision System ID

    def scan_and_detect_objects(self, object_ids):
        """
        Scan and detect multiple objects (Pipe and Trapezoid).
        :param object_ids: List of object IDs to detect (e.g., [1, 2])
        :return: Detected objects with their details
        """
        # 1️⃣ Step 1: Trigger the Scan
        self.pho_request_ls_scan(self.vs_id)
        print("Scanning environment...")

        # 2️⃣ Step 2: Request All Detected Objects (Assume a max limit, e.g., 10)
        response = self.pho_request_get_objects(self.vs_id, number_of_objects=10)

        # 3️⃣ Step 3: Filter the Required Objects (Pipe & Trapezoid)
        detected_objects = [obj for obj in response if obj['id'] in object_ids]

        # Display Detected Objects
        for obj in detected_objects:
            print(f"Detected: {obj['name']} (ID: {obj['id']})")
            print(f"  Position: {obj['position']}")
            print(f"  Orientation: {obj['orientation']}\n")

        return detected_objects

    # Existing Request Functions (Assumed)
    def pho_request_ls_scan(self, vs_id, tool_pose=None):
        payload = [vs_id, 0, 0, 0]
        if tool_pose:
            assert len(tool_pose) == 7, 'Invalid tool_pose size'
            payload += floatArray2bytes(tool_pose)
        self.pho_send_request(PHO_SCAN_LS_REQUEST, payload)

    def pho_request_get_objects(self, vs_id, number_of_objects):
        payload = [vs_id, 0, 0, 0] + [number_of_objects, 0, 0, 0]
        self.pho_send_request(PHO_GET_OBJECT_LS_REQUEST, payload)
        return self.pho_receive_response(PHO_GET_OBJECT_LS_REQUEST)

    # Placeholder for sending/receiving (replace with actual implementation)
    def pho_send_request(self, request_type, payload):
        print(f"Sending request: {request_type}, Payload: {payload}")

    def pho_receive_response(self, request_type):
        # Simulated Response (Replace with actual data)
        return [
            {"id": 1, "name": "Pipe", "position": [100, 200, 300], "orientation": [0, 0, 0, 1]},
            {"id": 2, "name": "Trapezoid", "position": [150, 250, 350], "orientation": [0, 0, 0, 1]},
            {"id": 3, "name": "Cube", "position": [200, 300, 400], "orientation": [0, 0, 0, 1]}
        ]


# 🚀 Example Usage
if __name__ == "__main__":
    vision_controller = VisionSystemController(vs_id=1)
    detected_objects = vision_controller.scan_and_detect_objects([1, 2])  # Detect Pipe & Trapezoid

