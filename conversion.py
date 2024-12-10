from neurapy.robot import Robot

def convert_quaternion_to_euler_pose(self, cartesian_pose):
    """
    Convert a pose with quaternion representation to a pose with Euler angles.

    If the input pose is already in Euler format (length 6), it is returned unchanged.

    :param cartesian_pose: The pose to be converted to Euler angles ([X, Y, Z, W, EX, EY, EZ], required: Yes).
    :type cartesian_pose: list

    :return: The pose in Euler angle format ([X, Y, Z, R, P, Y]).
    :rtype: list

    **Sample Usage:**
    
    """


    r = Robot()
    quaternion_pose = [0.086,0.023,0.907,0.979,-0.058,-0.177,0.079]  # [X, Y, Z, W, EX, EY, EZ]
    euler_pose = r.convert_quaternion_to_euler_pose(quaternion_pose)
    print(euler_pose)  # Output: [X, Y, Z, R, P, Y] with Euler angle values.