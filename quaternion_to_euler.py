
from neurapy.robot import Robot

r = Robot() # setting the RObot class

def convert_quaternion_to_euler_pose(): # defining the function for convertin the quaternion to euler


    quaternion_pose = [-0.431,-0.467,0.067,0.016,-0.4,0.916,-0.033] # [X, Y, Z, W, EX, EY, EZ]
    euler_pose = r.convert_quaternion_to_euler_pose(quaternion_pose) # getting te euler pose
    print(euler_pose)  # Output: [X, Y, Z, R, P, Y] with Euler angle values.

convert_quaternion_to_euler_pose() # calling the function
 

# [-0.431, -0.447, 0.067, 0.0733116517009277, 3.138682979903034, 0.8232587543719854]



# Converted Euler Pose: [-0.4314776916503906, -0.466967529296875, 0.06689299774169923, 2.31756399805644, -3.137481442024678, 0.07435508841430138]