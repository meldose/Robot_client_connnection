
from neurapy.robot import Robot # import the robot class
r = Robot() # create an instance of the robot class
target_angle = r.ik_fk("ik", target_pose =[-0.495, -0.307, 0.055, 0.07675785323856764, -3.122286946395002, 1.0561112514591882], # object pose values in radians
# current_joint=[0.4129863573167266, -0.040367615708926226, -1.603394842405772,-0.07106004628138933, -1.5406368975914917, -2.2229109243340384])
current_joint=r.get_current_joint_angles())


print(target_angle) # print the target joint angles


# from neurapy.robot import Robot # import the robot class
# r = Robot() # create an instance of the robot class
# target_angle = r.ik_fk("ik", target_pose =[x,y,z,d,a,b,c], # object pose values in radians
# current_joint = r.get_current_joint_angles()) # current joint angles in radians
# print(target_angle) # print the target joint angles
