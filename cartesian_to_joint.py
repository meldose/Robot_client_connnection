
from neurapy.robot import Robot # import the robot class
r = Robot() # create an instance of the robot class
target_angle = r.ik_fk("ik", target_pose =[-0.422, -0.337, 0.057, -0.0203424580285813, -2.890357129037366, 1.5576431237169093], # object pose values in radians
current_joint = [0.4129184862608269,-0.04035147853479624,-1.6033459562606136,-0.07107998043766754,-1.5406373722142601,0.910522489241973]) # current joint angles in radians
print(target_angle) # print the target joint angles


# from neurapy.robot import Robot # import the robot class
# r = Robot() # create an instance of the robot class
# target_angle = r.ik_fk("ik", target_pose =[x,y,z,d,a,b,c], # object pose values in radians
# current_joint = r.get_current_joint_angles()) # current joint angles in radians
# print(target_angle) # print the target joint angles
