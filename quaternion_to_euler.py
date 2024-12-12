from neurapy.robot import Robot

def convert_quaternion_to_euler_pose():

    r = Robot()
    quaternion_pose = [-0.519,-0.362,0.055,0.235,0.063,-0.839,0.486]  # [X, Y, Z, W, EX, EY, EZ]
    euler_pose = r.convert_quaternion_to_euler_pose(quaternion_pose)
    print(euler_pose)  # Output: [X, Y, Z, R, P, Y] with Euler angle values.

convert_quaternion_to_euler_pose()


