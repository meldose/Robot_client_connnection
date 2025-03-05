class VisionSystemController: # defining class Visioncontroller
    def __init__(self, vs_id): # initialising the vs_id
        self.vs_id = vs_id # Vision System ID

    def scan_and_detect_objects(vs_id, object_ids): # defining the function for scaning and detecting the objects

        # param object_ids: List of object IDs to detect (e.g., [1, 2])
        # return: Detected objects with their details

        #Step 1: Trigger the Scan
        self.pho_request_ls_scan(vs_id) # triggering the camera for scanning the objects
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
        payload = [vs_id, 0, 0, 0] + [number_of_objects, 0, 0, 0] # setting up the payload with vs_id and number of objects
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
        
# 🚀 Example Usage
if __name__ == "__main__":
    vision_controller = VisionSystemController(vs_id)
    detected_objects = vision_controller.scan_and_detect_objects([1, 2])  # Detect Pipe & Trapezoid
    
# Simulated response from vision system
object_data = [
    {"id": 2, "name": "object_B", "position": [0.5, 0.3, 0.1], "orientation": [0, 0, 0, 1]},
    {"id": 1, "name": "object_A", "position": [0.2, 0.4, 0.1], "orientation": [0, 0, 0, 1]},
]

# Sort objects by ID
sorted_objects = sorted(object_data, key=lambda obj: obj["id"])

# Function to scan objects
def scan_objects():
    print("Scanning objects...")
    return sorted_objects  # Simulated response from camera

# Function to pick an object
def pick_object(obj):
    print(f"Picking object {obj['name']} with ID {obj['id']}")
    # Move robot to object position and pick
    move_robot_to_position(obj["position"], obj["orientation"])
    control_gripper("close")  # Simulated gripper action

# Main logic
objects_to_pick = scan_objects()
for obj in objects_to_pick:
    pick_object(obj)
