from neurapy.robot import Robot

def convert_quaternion_to_euler_pose():

    r = Robot()
    quaternion_pose = [-0.422,-0.337,0.057,0.096,0.695,-0.707,0.0807] # [X, Y, Z, W, EX, EY, EZ]
    euler_pose = r.convert_quaternion_to_euler_pose(quaternion_pose)
    print(euler_pose)  # Output: [X, Y, Z, R, P, Y] with Euler angle values.

convert_quaternion_to_euler_pose()


