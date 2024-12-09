#  Quaternion to roll pitch yaw

    from neurapy.robot import Robot
    r = Robot()
    rpy = r.quaternion_to_rpy(0.85,0,0.52,0)

#   Roll pitch yaw to quaternion


    from neurapy.robot import Robot
    r = Robot()
    quaternion = r.rpy_to_quaternion(0,1.1,0)
    
    
### Quaternion to roll pitch yaw



    from neurapy.robot import Robot
    r = Robot()
    rpy = r.quaternion_to_rpy(0.85,0,0.52,0)

### Roll pitch yaw to quaternion



    from neurapy.robot import Robot
    r = Robot()
    quaternion = r.rpy_to_quaternion(0,1.1,0)

 ################################################################################
import neurapy.motion_planner as mp


def rpy_to_quaternion(r, p, y):
    rotation = mp.Rotation(r, p, y)
    return list(rotation.toQuaterniond())


def quaternion_to_rpy(w, x, y, z):
    rotation = mp.Rotation(w, x, y, z)
    return [value[0] for value in rotation.toRollPitchYaw()]


##################################################################################


# Working
import pytest
from numpy import allclose

from neurapy.helpers import quaternion_to_rpy, rpy_to_quaternion


@pytest.mark.run(order=1)
def test_rpy_to_quaternion():
    quaternion = rpy_to_quaternion(0, 1.1, 0)
    result = allclose(quaternion, [0.85, 0, 0.52, 0], atol=0.01)
    assert result == True

@pytest.mark.run(order=2)
def test_quaternion_to_rpy():
    rpy = quaternion_to_rpy(0.85, 0, 0.52, 0)
    result = allclose(rpy, [0, 1.1, 0], atol=0.01)
    assert result == True

######################################################################################

