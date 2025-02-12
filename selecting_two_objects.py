class VisionSystemController:
    def __init__(self, vs_id):
        self.vs_id = vs_id # Vision System ID

    def scan_and_detect_objects(self, object_ids):
        """

        :param object_ids: List of object IDs to detect (e.g., [1, 2])
        :return: Detected objects with their details
        """
        #Step 1: Trigger the Scan
        self.pho_request_ls_scan(vs_id)
        print("Scanning environment...")

        # Step 2: Request All Detected Objects (Assume a max limit, e.g., 10)
        response = self.pho_request_get_objects(vs_id, number_of_objects=2)

        # Step 3: Filter the Required Objects (Pipe & Trapezoid)
        detected_objects = [obj for obj in response if obj['id'] in object_ids]

        # Display Detected Objects
        for obj in detected_objects:
            print(f"Detected: {obj['name']} (ID: {obj['id']})") 
            print(f"  Position: {obj['position']}")
            print(f"  Orientation: {obj['orientation']}\n")
        return detected_objects

    # Existing Request Functions (Assumed)
    def pho_request_ls_scan(self, vs_id, tool_pose=None):
        
        if tool_pose is None:
            
            payload = [vs_id, 0, 0, 0]  # payload - vision system id
            self.pho_send_request(PHO_SCAN_LS_REQUEST, payload)
        else:
            
            assert len(tool_pose) == 7, 'Wrong tool_pose size'
            payload = [vs_id, 0, 0, 0]  # payload - vision system id
            payload = payload + floatArray2bytes(tool_pose)  # payload - start
            self.pho_send_request(PHO_SCAN_LS_REQUEST, payload)

    def pho_request_get_objects(self, vs_id,  number_of_objects):
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
            {"id": 2, "name": "Trapezoid", "position": [150, 250, 350], "orientation": [0, 0, 0, 1]}
        ]
    def floatArray2bytes(array): # function to convert float array to bytes
        msg = [] # creating the message
        for value in array: # iterating through the array
            msg = msg + list(struct.pack('<f', value)) # converting to bytes
        return msg # returning the message


    def build_hello_msg(): # function to build the hello message
        return bytearray(BRAND_IDENTIFICATION.encode('utf-8')) # returning the message


    def build_state_server_hello_msg(): # function to build the state server hello message
        return bytearray(BRAND_IDENTIFICATION_SERVER.encode('utf-8')) # returning the message


# 🚀 Example Usage
if __name__ == "__main__":
    vision_controller = VisionSystemController(vs_id=1)
    detected_objects = vision_controller.scan_and_detect_objects([1, 2])  # Detect Pipe & Trapezoid

