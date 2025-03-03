
from neurapy.robot import Robot

def convert_quaternion_to_euler_pose(): # defining the function for convertin the quaternion to euler

    r = Robot() # setting the RObot class
    # quaternion_pose = new_message # [X, Y, Z, W, EX, EY, EZ]
    quaternion_pose = [-0.495,-0.307,0.055,0.012,-0.519,0.854,-0.038]
    euler_pose = r.convert_quaternion_to_euler_pose(quaternion_pose) # getting te euler pose
    print(euler_pose)  # Output: [X, Y, Z, R, P, Y] with Euler angle values.

convert_quaternion_to_euler_pose() # calling the function
 