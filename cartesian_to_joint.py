
from neurapy.robot import Robot
r = Robot()
# target_pose = r.ik_fk("fk", target_angle = [0.2,0.2,0.2,0.2,0.2,0.2])
target_angle = r.ik_fk("ik", target_pose = [-0.497,-0.179,0.345,3.14,0,-1.57],
current_joint = [-2.80,0.32,1.39,0.04,1.49,1.66])
print(target_angle)