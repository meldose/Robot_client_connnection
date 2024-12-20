from neurapy.robot import Robot

def convert_quaternion_to_euler_pose():

    r = Robot()
    quaternion_pose = [-0.576,-0.43,0.234,-3.02,-0.06,1.41] # [X, Y, Z, W, EX, EY, EZ]
    euler_pose = r.convert_quaternion_to_euler_pose(quaternion_pose)
    print(euler_pose)  # Output: [X, Y, Z, R, P, Y] with Euler angle values.

convert_quaternion_to_euler_pose()


