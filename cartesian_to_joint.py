
from neurapy.robot import Robot
r = Robot()
# target_pose = r.ik_fk("fk", target_angle = [0.2,0.2,0.2,0.2,0.2,0.2])
target_angle = r.ik_fk("ik", target_pose = [-0.785,-0.105,0.20,3.14,0,-1.57],
current_joint = [-3.0,0.06,0.26,-0.0,1.38,1.71])
print(target_angle)