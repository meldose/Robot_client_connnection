
from neurapy.robot import Robot

r = Robot() # setting the RObot class

def convert_quaternion_to_euler_pose(): # defining the function for convertin the quaternion to euler


    quaternion_pose = [-0.431,-0.467,0.067,0.016,-0.4,0.916,-0.033] # [X, Y, Z, W, EX, EY, EZ]
    euler_pose = r.convert_quaternion_to_euler_pose(quaternion_pose) # getting te euler pose
    print(euler_pose)  # Output: [X, Y, Z, R, P, Y] with Euler angle values.

convert_quaternion_to_euler_pose() # calling the function
 




# [-0.431, -0.467, 0.067, 0.0733116517009277, 3.138682979903034, 0.8232587543719854]

# Converted Euler Pose: [-0.4314786682128906, -0.4669829406738281, 0.06690879058837891, 0.07439787017193546, 3.1375300055789515, 0.8239746638206142]