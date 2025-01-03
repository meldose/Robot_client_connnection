from neurapy.robot import Robot

def convert_quaternion_to_euler_pose():

    r = Robot()
    quaternion_pose = [-0.508,-0.389,0.110,0.079,0.852,0.517,-0.019] # [X, Y, Z, W, EX, EY, EZ]
    euler_pose = r.convert_quaternion_to_euler_pose(quaternion_pose)
    print(euler_pose)  # Output: [X, Y, Z, R, P, Y] with Euler angle values.

convert_quaternion_to_euler_pose()


