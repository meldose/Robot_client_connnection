import time
import cv2  # Assuming you're using OpenCV to interface with the camera

def capture_frame_with_timestamp(camera_index=0):
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return None, None
    
    ret, frame = cap.read()
    timestamp = time.time()  # Capture the timestamp in seconds
    
    cap.release()
    
    if ret:
        return frame, timestamp
    else:
        print("Error: Could not read frame.")
        return None, None

# Example usage
frame, timestamp = capture_frame_with_timestamp()
if frame is not None:
    print(f"Captured frame at timestamp: {timestamp}")
