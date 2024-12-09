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

import os
SOCKET_INTERFACE = os.getenv('SOCKET_INTERFACE',"false").lower()=='true'
if SOCKET_INTERFACE:
    from neurapy.socket_interface.robot import Robot
else:
    from neurapy.robot import Robot

r = Robot()


def test_rpy_to_quaternion():
    quat = r.rpy_to_quaternion(0,0,0)
    assert quat == [1.0,0,0,0]
    
    
#######################################################################################

import os
SOCKET_INTERFACE = os.getenv('SOCKET_INTERFACE',"false").lower()=='true'
if SOCKET_INTERFACE:
    from neurapy.socket_interface.robot import Robot
else:
    from neurapy.robot import Robot

r = Robot()


def test_quaternion_to_rpy():
    assert r.quaternion_to_rpy(1,0,0,0) == [0,0,0]
    
    
#########################################################################################


def convert_quaternion_to_euler_pose(self, cartesian_pose : list) -> list:
    """
    Converts a pose with quaternion to a pose with euler angles.
    If pose with euler angles (length 6) is given as parameters, the pose is returned without making changes.

    Parameters
    ----------
    - cartesian_pose : list
        - the pose which should be converted to euler angles
        - allowed format: [X,Y,Z,R,P,Y] and [X,Y,Z,W,EX,EY,EZ]

    Returns
    ----------
    - list:
        - the pose with euler angles
        - format: [X,Y,Z,R,P,Y]
    """
    cartesian_pose = np.array(cartesian_pose).flatten().tolist()
    if len(cartesian_pose) == 6:
        return cartesian_pose

    euler_cartesian_pose = np.zeros(6).tolist()
    euler_cartesian_pose[:3] = cartesian_pose[:3]
    euler_cartesian_pose[3:] = self.robot.quaternion_to_rpy(*cartesian_pose[3:])
    return np.array(euler_cartesian_pose).flatten().tolist()

def convert_euler_to_quaternion_pose(self, cartesian_pose : list) -> list:
    """
    Converts a pose with euler angles to quaternions.
    If a pose with quaternions (length 7) is given as parameter, the pose is returned without making changes. 

    Parameters
    ----------
    - cartesian_pose : list
        - the pose which should be converted to quaternion
        - allowed formate: [X,Y,Z,R,P,Y] and [X,Y,Z,W,EX,EY,EZ]

    Returns
    ----------
    - list:
        - the pose with quaternion
        - format: [X,Y,Z,W,EX,EY,EZ]
    """
    cartesian_pose = np.array(cartesian_pose).flatten().tolist()
    if len(cartesian_pose) == 7:
        return cartesian_pose
    
    quaternion_cartesian_pose = np.zeros(7).tolist()
    quaternion_cartesian_pose[:3] = cartesian_pose[:3]
    quaternion_cartesian_pose[3:] = self.robot.rpy_to_quaternion(*cartesian_pose[3:])
    return np.array(quaternion_cartesian_pose).flatten().tolist()

def compute_forwards_kinematic(self, joint_angles : list) -> tuple:
    """
    Computes the forward kinematics for the given angles

    Parameters
    ----------
    - joint_angles : list
        - the joint angles which should be converted
        - allowed formate: 
            - dof 6: [a1, a2, a3, a4, a5, a6]
            - dof 7: [a1, a2, a3, a4, a5, a6, a7]

    Returns 
    ----------
    bool, list
    - bool: is_fk_found
        - True if forwards kinematic was able to be found, else False
    - list: cartesian_pose
        - The catesian pose which was found with the forwards kinematic
        - formate: [X,Y,Z,W,EX,EY,EZ]
    """
    is_fk_found = False
    result_pose = np.zeros(7).tolist()
    try:
        result_pose = self.robot.ik_server.getFKQuaternion(np.array(joint_angles))
        is_fk_found = not np.isnan(np.linalg.norm(np.array(result_pose)))
    except:
        pass
    return is_fk_found, np.array(result_pose).flatten().tolist()

def compute_inverse_kinematic(self, joint_angles : list, cartesian_pose : list) -> list:
    """
    Computes the inverse kinematic (IK) of the cartesian pose, with the joint angles as reference.
    This method tries multiple times to find a inverse kinematic, with first adding noice to the 
    cartesian pose and then adding noice to the joint angles.

    Parameters
    ----------
    - joint_angles : list
        - the reference joint angles, from which the joint configuration needs to be searched from
        - formate:
            - dof 6: [a1, a2, a3, a4, a5, a6]
            - dof 7: [a1, a2, a3, a4, a5, a6, a7]
    - cartesian_pose : list
        - the cartesian pose, from which the IK should be calculated from
        - allowed formate: pose with euler angles [X,Y,Z,R,P,Y] and pose with quaternion [X,Y,Z,W,EX,EY,EZ]

    Returns
    ----------
    - list
        - the found joint configuration which correlates to the cartesian pose
        - formate:
            - dof 6: [a1, a2, a3, a4, a5, a6]
            - dof 7: [a1, a2, a3, a4, a5, a6, a7]

    Throws
    ----------
    - Exception
        - if no IK was found
    """
    noise_generator = np.random.default_rng()
    reference_joint_angles = np.array(joint_angles)

    quat_reference_pose = np.array(self.convert_euler_to_quaternion_pose(cartesian_pose))
    is_fk_found, fk_result = self.compute_forwards_kinematic(reference_joint_angles.tolist())
    fk_result = np.array(fk_result)

    is_orientation_correct = np.allclose(quat_reference_pose[3:], fk_result[3:],atol=1e-2) or np.allclose(-1 * quat_reference_pose[3:], fk_result[3:],atol=1e-2)
    is_translation_correct = np.allclose(quat_reference_pose[:3],fk_result[:3],atol=1e-3)

    if is_fk_found and is_translation_correct and is_orientation_correct:
        return reference_joint_angles.tolist()

    euler_reference_pose = self.convert_quaternion_to_euler_pose(cartesian_pose)

    for joint_iterations in range(0,4):
        for pose_iterations in range(0,5):
            try:
                result_joint_angles = self.robot.ik_fk("ik", target_pose = list(euler_reference_pose), current_joint = list(reference_joint_angles), tool_params = self.robot.tool_params)
                if not np.isnan(np.linalg.norm(np.array(result_joint_angles))):
                    return np.array(result_joint_angles).tolist()
            except:
                pass
            noise_cart_array = (((noise_generator.random(3) - 0.5) * 2) * 1e-5)
            noise_cart_array.resize(np.array(cartesian_pose).shape[0])
            euler_reference_pose = np.array(cartesian_pose) + noise_cart_array
        noise_joint_array = (((noise_generator.random(self.robot.dof) - 0.5) * 2) * 1e-5)
        reference_joint_angles = np.array(joint_angles) + noise_joint_array

    message = str(
        self.__class__.__name__ 
        + " - Point is not reachable with the selected tool" 
        + ", check the point or tool configuration."
    )
    self.robot.logger.error(message)
    raise Exception("Error " + message)

#######################################################################################################################
