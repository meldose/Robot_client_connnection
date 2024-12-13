
from neurapy.robot import Robot
def convert_euler_to_quaternion_pose():

    r = Robot()
    euler_pose = [-467,-345,208,-3.08,0.0159,-2.3]  # [X, Y, Z, R, P, Y]
    quaternion_pose = r.convert_euler_to_quaternion_pose(euler_pose)
    print(quaternion_pose)  # Output: [X, Y, Z, W, EX, EY, EZ] with quaternion values.

convert_euler_to_quaternion_pose()