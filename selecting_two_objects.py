class VisionSystemController: # defining class Visioncontroller
    def __init__(self, vs_id): # initialising the vs_id
        self.vs_id = vs_id # Vision System ID

    def scan_and_detect_objects(vs_id, object_ids): # defining the function for scaning and detecting the objects

        # param object_ids: List of object IDs to detect (e.g., [1, 2])
        # return: Detected objects with their details

        #Step 1: Trigger the Scan
        self.pho_request_ls_scan(vs_id)
        print("Scanning environment...") # printing the comment for scanning the environment

        # Step 2: Request All Detected Objects (Assume a max limit, e.g., 10)
        response = self.pho_request_get_objects(vs_id, number_of_objects=2) # creating the varibale for assigning the vs_id and number of objects

        # Step 3: Filter the Required Objects (Pipe & Trapezoid)
        detected_objects = [obj for obj in response if obj['id'] in object_ids] # checking the detected the objects

        # Display Detected Objects
        for obj in detected_objects: # checking the detected objects in the for loop
            print(f"Detected: {obj['name']} (ID: {obj['id']})") # if detected print the object name and ID
            print(f"  Position: {obj['position']}") # print the position 
            print(f"  Orientation: {obj['orientation']}\n") # print the orientation
        return detected_objects # get the detected objects

    # Existing Request Functions (Assumed)
    def pho_request_ls_scan(self, vs_id, tool_pose=None): # defining the function for request for locator scan
        
        if tool_pose is None: # checking if the tool pose is None or not
            
            payload = [vs_id, 0, 0, 0]  # payload - vision system id
            self.pho_send_request(PHO_SCAN_LS_REQUEST, payload) # sending the request to the photoneo camera
            
        else:
            
            assert len(tool_pose) == 7, 'Wrong tool_pose size' #  chceking if the lenght is equal to seven or not 
            payload = [vs_id, 0, 0, 0]  # payload - vision system id
            payload = payload + floatArray2bytes(tool_pose)  # payload - start
            self.pho_send_request(PHO_SCAN_LS_REQUEST, payload) # sending the Photoneo reuqest

    def pho_request_get_objects(self, vs_id,  number_of_objects): # function for request to get the objects that contains the vs_id and number of objects
        payload = [vs_id, 0, 0, 0] + [number_of_objects, 0, 0, 0]
        self.pho_send_request(PHO_GET_OBJECT_LS_REQUEST, payload) # sending the request from the camera
        return self.pho_receive_response(PHO_GET_OBJECT_LS_REQUEST) # photoneo recieving the response

    # Placeholder for sending/receiving (replace with actual implementation)
    def pho_send_request(self, request_type, payload): # defining the function for sending the request sss
        print(f"Sending request: {request_type}, Payload: {payload}") # print the request_type and payload

    def pho_receive_response(self, request_type): # defining the function for recieving the response
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
    
