
from neurapy.robot import Robot # import the robot class
r = Robot() # create an instance of the robot class
target_angle = r.ik_fk("ik", target_pose =[-0.499,0.324,0.057,0.014,0.511,0.858,0.039], # object pose values in radians
# current_joint = [0.4129184862608269,-0.04035147853479624,-1.6033459562606136,-0.07107998043766754,-1.5406373722142601,0.910522489241973]) # current joint angles in radians
current_joint=[0.4129863573167266, -0.040367615708926226, -1.603394842405772,-0.07106004628138933, -1.5406368975914917, -2.2229109243340384])


print(target_angle) # print the target joint angles


# from neurapy.robot import Robot # import the robot class
# r = Robot() # create an instance of the robot class
# target_angle = r.ik_fk("ik", target_pose =[x,y,z,d,a,b,c], # object pose values in radians
# current_joint = r.get_current_joint_angles()) # current joint angles in radians
# print(target_angle) # print the target joint angles
