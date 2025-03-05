
from neurapy.robot import Robot

r = Robot() # setting the RObot class

def convert_quaternion_to_euler_pose(): # defining the function for convertin the quaternion to euler


    quaternion_pose = [-0.541,-0.355,0.055,0.01,-0.677,0.735,-0.043] # [X, Y, Z, W, EX, EY, EZ]
    euler_pose = r.convert_quaternion_to_euler_pose(quaternion_pose) # getting te euler pose
    print(euler_pose)  # Output: [X, Y, Z, R, P, Y] with Euler angle values.

convert_quaternion_to_euler_pose() # calling the function


