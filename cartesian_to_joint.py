
from neurapy.robot import Robot
r = Robot()
# target_pose = r.ik_fk("fk", target_angle = [0.2,0.2,0.2,0.2,0.2,0.2])
target_angle = r.ik_fk("ik", target_pose = [-0.785,-0.105,0.154,3.14,0,-1.57], # object pose values in radians
current_joint = [-3.01,1.23,0.27,-0,1.40,1.71]) # current joint angles in radians
print(target_angle)