
from neurapy.robot import Robot
r = Robot()
# target_pose = r.ik_fk("fk", target_angle = [0.2,0.2,0.2,0.2,0.2,0.2])



target_angle = r.ik_fk("ik", target_pose =[-0.511, -0.367, 0.241, -3.02,-0.06,1.41], # object pose values in radians
current_joint = [0.63,0.35,-1.34,3.01,1.15,-0.76]) # current joint angles in radians
print(target_angle)



