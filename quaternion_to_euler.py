
from neurapy.robot import Robot

def convert_quaternion_to_euler_pose(): # defining the function for convertin the quaternion to euler

    r = Robot() # setting the RObot class
    # quaternion_pose = new_message # [X, Y, Z, W, EX, EY, EZ]
    quaternion_pose = [-0.564,-0.383,0.055,0.019,-0.491,0.87,-0.037]
    euler_pose = r.convert_quaternion_to_euler_pose(quaternion_pose) # getting te euler pose
    print(euler_pose)  # Output: [X, Y, Z, R, P, Y] with Euler angle values.

convert_quaternion_to_euler_pose() # calling the function
 