
from neurapy.robot import Robot
r = Robot()
# target_pose = r.ik_fk("fk", target_angle = [0.2,0.2,0.2,0.2,0.2,0.2])
target_angle = r.ik_fk("ik", target_pose = [-0.55,-0.489,-0.172,-2.5,-0.004,2.33], # object pose values in radians
current_joint = [-2.42,0.42,0.88,-0.06,0.26,-1.57]) # current joint angles in radians
print(target_angle)



