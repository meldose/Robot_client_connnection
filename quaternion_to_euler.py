from neurapy.robot import Robot

def convert_quaternion_to_euler_pose():

    r = Robot()
    quaternion_pose = [-0.520,-0.410,0.234,0.004,-0.428,0.904,-0.007] # [X, Y, Z, W, EX, EY, EZ]
    euler_pose = r.convert_quaternion_to_euler_pose(quaternion_pose)
    print(euler_pose)  # Output: [X, Y, Z, R, P, Y] with Euler angle values.

convert_quaternion_to_euler_pose()


