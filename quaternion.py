#  Quaternion to roll pitch yaw

    from neurapy.robot import Robot
    r = Robot()
    rpy = r.quaternion_to_rpy(0.85,0,0.52,0)

#   Roll pitch yaw to quaternion


    from neurapy.robot import Robot
    r = Robot()
    quaternion = r.rpy_to_quaternion(0,1.1,0)