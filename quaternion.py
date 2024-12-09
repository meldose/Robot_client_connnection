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


#Working --- but it returns the values, not true or false
import os
SOCKET_INTERFACE = os.getenv('SOCKET_INTERFACE',"false").lower()=='true'
if SOCKET_INTERFACE:
    from neurapy.socket_interface.robot import Robot
else:
    from neurapy.robot import Robot
    
import pytest
from neurapy.exceptions import IKNotFound

r = Robot()
target_joint_angles=[0.2,0.2,0.2,0.2,0.2,0.2]
end_effector_pose = [0.31937441035446845, 0.07106314399820793, 1.109090727533707, 0.2218226471738111, 0.5592622696480269, 0.672183900379411]
@pytest.mark.skip #function is deprecated
def test_ik_fk():
    target_pose = [round(x,1) for x in r.ik_fk("fk", target_angle = target_joint_angles)]
    target_angle = r.ik_fk("ik", target_pose = end_effector_pose, current_joint = target_joint_angles)
    assert target_pose is not None
    assert target_angle is not None
@pytest.mark.skip #function is deprecated
def test_ik_not_found():
    target_pose = [0.0,0.0,3.0,0.0,0.0,0.0]
    reference_angle = [0.0]*6
    if SOCKET_INTERFACE:
        with pytest.raises(Exception):
            result = r.compute_inverse_kinematics(target_pose,reference_angle)
    else:
        with pytest.raises(IKNotFound):
            result = r.compute_inverse_kinematics(target_pose,reference_angle)
            
##################################################################################################


#### Forward/Inverse kinematics

from neurapy.robot import Robot
r = Robot()
target_pose = r.ik_fk("fk", target_angle = [0.2,0.2,0.2,0.2,0.2,0.2])
target_angle = r.ik_fk("ik", target_pose = [0.140448, -0.134195, 1.197456, 3.1396, -0.589, -1.025],current_joint = [-1.55, -0.69, 0.06, 1.67, -0.02, -1.57, 0.11])

########### EXAMPLE FROM CHATGPT #############################################################
from neurapy.robot import Robot

r= Robot()

import numpy as np
from scipy.spatial.transform import Rotation as R

def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Convert Euler angles to a rotation matrix.
    Roll, pitch, yaw are in radians.
    """
    rotation = R.from_euler('xyz', [roll, pitch, yaw])
    return rotation.as_matrix()


# Initialize the robot
robot = Robot('your_robot')

# Define target coordinates and orientation (example values)
x, y, z = 0.5, 0.0, 0.2  # Position in meters
roll, pitch, yaw = 0.0, 0.0, 0.0  # Orientation in radians


def get_target_pose(x, y, z, roll, pitch, yaw):
    rotation_matrix = euler_to_rotation_matrix(roll, pitch, yaw)
    position = np.array([x, y, z])
    
    # Create a 4x4 homogeneous transformation matrix
    pose = np.eye(4)
    pose[:3, :3] = rotation_matrix
    pose[:3, 3] = position
    return pose


# Get the target pose
target_pose = get_target_pose(x, y, z, roll, pitch, yaw)

ik_solution = robot.arm.inverse_kinematics(target_pose)

if ik_solution is not None:
    try:
        robot.arm.set_joint_positions(ik_solution)
        print("Robot successfully moved to the target pose.")
    except Exception as e:
        print(f"Failed to move the robot: {e}")
else:
    print("No valid IK solution found for the target pose.")
    
    

#########################################################################################

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from math import radians
from scipy.spatial.transform import Rotation as R

def euler_to_quaternion(roll, pitch, yaw):
    rotation = R.from_euler('xyz', [roll, pitch, yaw])
    return rotation.as_quat()

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_target_pose', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"  # Replace with your robot's MoveIt! group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Define target position and orientation
    target_position = [0.5, 0.0, 0.2]
    roll, pitch, yaw = 0.0, 0.0, 0.0
    target_orientation = euler_to_quaternion(roll, pitch, yaw)

    # Create a pose message
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = target_position[0]
    pose_target.position.y = target_position[1]
    pose_target.position.z = target_position[2]
    pose_target.orientation.x = target_orientation[0]
    pose_target.orientation.y = target_orientation[1]
    pose_target.orientation.z = target_orientation[2]
    pose_target.orientation.w = target_orientation[3]

    move_group.set_pose_target(pose_target)

    # Plan and execute
    plan = move_group.go(wait=True)

    # Ensure the robot has reached the goal
    move_group.stop()
    move_group.clear_pose_targets()

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()


####################################################################################################

from abc import ABC, abstractmethod
import math
from typing import List, Optional
from scipy.spatial.transform import Rotation as R
import numpy as np


class Camera(ABC):

    def __init__(self):
        self.message = None
        self.error_type = None

    @abstractmethod
    def get_pose(self, *args, **kwargs) -> list:
        """
        Function to get robot pose from the camera
        """
        pass

    def active_matrix_from_angle(self, basis, angle):
        """Compute active rotation matrix from rotation about basis vector.

        With the angle :math:`\alpha` and :math:`s = \sin{\alpha}, c=\cos{\alpha}`,
        we construct rotation matrices about the basis vectors as follows:

        .. math::

            \boldsymbol{R}_x(\alpha) =
            \left(
            \begin{array}{ccc}
            1 & 0 & 0\\
            0 & c & -s\\
            0 & s & c
            \end{array}
            \right)

        .. math::

            \boldsymbol{R}_y(\alpha) =
            \left(
            \begin{array}{ccc}
            c & 0 & s\\
            0 & 1 & 0\\
            -s & 0 & c
            \end{array}
            \right)

        .. math::

            \boldsymbol{R}_z(\alpha) =
            \left(
            \begin{array}{ccc}
            c & -s & 0\\
            s & c & 0\\
            0 & 0 & 1
            \end{array}
            \right)

        Parameters
        ----------
        basis : int from [0, 1, 2]
            The rotation axis (0: x, 1: y, 2: z)

        angle : float
            Rotation angle

        Returns
        -------
        R : array, shape (3, 3)
            Rotation matrix

        Raises
        ------
        ValueError
            If basis is invalid
        """
        c = np.cos(angle)
        s = np.sin(angle)

        if basis == 0:
            R = np.array([[1.0, 0.0, 0.0],
                        [0.0, c, -s],
                        [0.0, s, c]])
        elif basis == 1:
            R = np.array([[c, 0.0, s],
                        [0.0, 1.0, 0.0],
                        [-s, 0.0, c]])
        elif basis == 2:
            R = np.array([[c, -s, 0.0],
                        [s, c, 0.0],
                        [0.0, 0.0, 1.0]])
        else:
            raise ValueError("Basis must be in [0, 1, 2]")

        return R
    
    def active_matrix_from_intrinsic_euler_xyz(self, e):
        """Compute active rotation matrix from intrinsic xyz Cardan angles.

        Parameters
        ----------
        e : array-like, shape (3,)
            Angles for rotation around x-, y'-, and z''-axes (intrinsic rotations)

        Returns
        -------
        R : array, shape (3, 3)
            Rotation matrix
        """
        alpha, beta, gamma = e
        R = self.active_matrix_from_angle(0, alpha).dot(
            self.active_matrix_from_angle(1, beta)).dot(
            self.active_matrix_from_angle(2, gamma))
        return R
    
    def frame_from_rotation_translation(self, r, t):
        """
        Given rotation matrix and translation vector, calculate the transformation matrix.
        
        Parameters:
        r (list): rotation matrix of shape (3,3)
        t (list): translation vector of shape (3,)
        
        Returns:
        numpy.ndarray: transformation matrix of shape (4,4)
        """
        arr = np.array(
            [
                [r[0][0], r[0][1], r[0][2], t[0]],
                [r[1][0], r[1][1], r[1][2], t[1]],
                [r[2][0], r[2][1], r[2][2], t[2]],
                [0, 0, 0, 1],
            ])
        return arr

    def pose_euler_to_mat(self, pose: List[float]) -> np.ndarray:
        rot = R.from_euler('xyz', pose[3:], degrees=False)
        pose_mat = np.eye(4)
        pose_mat[:3, 3] = np.array(pose[:3])
        pose_mat[:3, :3] = rot.as_matrix()
        return pose_mat

    def rotate_local(
        self,
        pose: List[float],
        x: Optional[float] = 0.0,
        y: Optional[float] = 0.0,
        z: Optional[float] = 0.0
        ) -> List[float]:
        """
        Rotate the pose in local coordinate frame (current pose coordinate frame)

        Parameters
        ----------
        pose : list
            The current pose
        x : float, optional
            The rotation angle around local axis x in rad, by default 0.0
        y : float, optional
            The rotation angle around local axis y in rad, by default 0.0
        z : float, optional
            The rotation angle around local axis z in rad, by default 0.0

        Returns
        -------
        Pose
            Rotated pose
        """
        pose_mat = self.pose_euler_to_mat(pose)
        rotation = self.active_matrix_from_intrinsic_euler_xyz([x, y, z])
        transformation = self.frame_from_rotation_translation(rotation, [0, 0, 0])
        transformed_mat = pose_mat.dot(transformation)
        transformed_rot = R.from_matrix(transformed_mat[:3, :3])
        transformed_pose = [*transformed_mat[:3, 3], *transformed_rot.as_euler('xyz', degrees=False)]
        return transformed_pose

    def check_camera_output(self, pose_from_camera: list) -> bool:
        """Function to check the format of camera output. It can be List[Float] or List[List[Float]].

        Arguments:
            pose_from_camera -- output from the camera 

        Returns:
            bool if format is correct or not
        """
        if all(isinstance(item, list) for item in pose_from_camera):
            return all([len(res)==6 and all(isinstance(x, (float, int)) for x in res) for res in pose_from_camera])
        else:
            return len(pose_from_camera)==6 and all(isinstance(x, (float, int)) for x in pose_from_camera)


    def reorient_camera_points(self, camera_point_list:list) -> list:
        """Function to rotate the given poses by 180 degrees to check for IK solution in order to avoid out of limit condition for last axis.

        Arguments:
            camera_point_list -- list of poses from the camera 

        Returns:
            Double the length of input list containing rotated poses
        """
        camera_points = []
        
        for pose in camera_point_list:
            camera_points.append(pose)
            if pose[5] >= 0:
                pose2 = self.rotate_local(pose, z=-math.pi)
            else:
                pose2 = self.rotate_local(pose, z=math.pi)
            camera_points.append(pose2)
        
        return camera_points

    def set_error_message(self, message:str, error_type:str) -> None:
        """
        Function to set error message to be send to GUI.

        Arguments:
            message -- The string you want to emit to GUI
            type -- Error or Warning
        """
        pass
    
    def get_error_message(self):
        """
        Function to get error message with type to be sent to GUI.

        Returns the type and the error message 
        """
        pass


######################################################################################################################

from abc import ABC, abstractmethod
import math
from typing import List, Optional
from scipy.spatial.transform import Rotation as R
import numpy as np
# Import your robot control library
# For example, using pyrobot
from pyrobot import Robot

# Assuming the Camera classes (RealCamera and SimulatedCamera) are defined as above

class RobotController:
    def __init__(self, robot: Robot, camera: Camera):
        """
        Initialize the RobotController with a robot and a camera.

        Parameters:
            robot (Robot): The robot instance.
            camera (Camera): The camera instance.
        """
        self.robot = robot
        self.camera = camera

    def target_robot_to_camera_pose(self):
        """
        Target the robot to the pose obtained from the camera.
        """
        # Get pose from the camera
        camera_pose = self.camera.get_pose()
        
        if not camera_pose:
            error_message = self.camera.get_error_message()
            print(f"Error retrieving pose: {error_message}")
            return
        
        # Optionally, reorient the camera points to handle IK constraints
        camera_pose_list = camera_pose if isinstance(camera_pose[0], list) else [camera_pose]
        valid_poses = self.camera.reorient_camera_points(camera_pose_list)
        
        for pose in valid_poses:
            if not self.camera.check_camera_output(pose):
                print(f"Invalid pose format: {pose}")
                continue
            
            # Convert pose to transformation matrix
            target_pose_mat = self.camera.pose_euler_to_mat(pose)
            
            # Extract position and orientation
            position = target_pose_mat[:3, 3]
            rotation_matrix = target_pose_mat[:3, :3]
            
            # Compute inverse kinematics
            ik_solution = self.robot.arm.inverse_kinematics(target_pose_mat)
            
            if ik_solution is not None:
                try:
                    # Move the robot's arm to the target joint angles
                    self.robot.arm.set_joint_positions(ik_solution)
                    print(f"Robot moved to pose: {pose}")
                except Exception as e:
                    print(f"Failed to move the robot: {e}")
            else:
                print(f"No IK solution found for pose: {pose}")

def main():
    # Initialize the robot (replace 'your_robot' with your actual robot name)
    robot = Robot('your_robot')

    sample_poses = [
        [0.5, 0.0, 0.2, 0.0, 0.0, 0.0],
        [0.6, 0.1, 0.3, 0.1, 0.0, 0.0],
        # Add more poses as needed
    ]
    camera = SimulatedCamera(predefined_poses=sample_poses)
    
    # Initialize the controller
    controller = RobotController(robot, camera)
    
    # Target the robot to each pose from the camera
    controller.target_robot_to_camera_pose()

if __name__ == "__main__":
    main()

#################################################################################################################