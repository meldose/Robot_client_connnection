
from neurapy.robot import Robot
r = Robot()
# target_pose = r.ik_fk("fk", target_angle = [0.2,0.2,0.2,0.2,0.2,0.2])



target_angle = r.ik_fk("ik", target_pose =[-0.513,-0.355,0.182,-3.04,-0.121,2.24], # object pose values in radians
current_joint = [0.63,0.28,-1.34,3.01,1.06,-1.57]) # current joint angles in radians
print(target_angle)



