
from neurapy.robot import Robot
r = Robot()
# target_pose = r.ik_fk("fk", target_angle = [0.2,0.2,0.2,0.2,0.2,0.2])
target_angle = r.ik_fk("ik", target_pose = [-0.785,-0.105,0.004,3.14,0,-1.57],
current_joint = [-2.89,0.98,0.59,-0.01,1.65,1.80])
print(target_angle)