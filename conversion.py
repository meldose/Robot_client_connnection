from neurapy.robot import Robot

def convert_quaternion_to_euler_pose():
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
    quaternion_pose = [-0.519,-0.362,0.055,0.235,0.063,-0.839,0.486]  # [X, Y, Z, W, EX, EY, EZ]
    euler_pose = r.convert_quaternion_to_euler_pose(quaternion_pose)
    print(euler_pose)  # Output: [X, Y, Z, R, P, Y] with Euler angle values.

convert_quaternion_to_euler_pose()


# from neurapy.robot import Robot

# def convert_euler_to_quaternion_pose(self,cartesian_pose):


#     """
#     Convert a pose with Euler angles to quaternions.

#     If the input pose is already in quaternion format (length 7), it is returned unchanged.

#     :param cartesian_pose: The pose to be converted to quaternion format ([X, Y, Z, R, P, Y], required: Yes).
#     :type cartesian_pose: list

#     :return: The pose in quaternion format ([X, Y, Z, W, EX, EY, EZ]).
#     :rtype: list

#     """"

#     r = Robot()
#     euler_pose = [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]  # [X, Y, Z, R, P, Y]
#     quaternion_pose = r.convert_euler_to_quaternion_pose(euler_pose)
#     print(quaternion_pose)  # Output: [X, Y, Z, W, EX, EY, EZ] with quaternion values.
    
#     if len(cartesian_pose) == 7:
#         return cartesian_pose
#     return cartesian_pose[:3] +  self.rpy_to_quaternion(cartesian_pose[3],cartesian_pose[4],cartesian_pose[5])

# convert_euler_to_quaternion_pose()