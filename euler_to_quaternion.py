
from neurapy.robot import Robot
def convert_euler_to_quaternion_pose():

    r = Robot()
    euler_pose = [-0.519,-0.362,0.182,-3.04,-0.121,2.24] # [X, Y, Z, R, P, Y]
    quaternion_pose = r.convert_euler_to_quaternion_pose(euler_pose)
    print(quaternion_pose)  # Output: [X, Y, Z, W, EX, EY, EZ] with quaternion values.

convert_euler_to_quaternion_pose()