
from neurapy.robot import Robot
r = Robot()
# target_pose = r.ik_fk("fk", target_angle = [0.2,0.2,0.2,0.2,0.2,0.2])
target_angle = r.ik_fk("ik", target_pose =[-0.519, -0.362, 0.055, -2.058652790827435, -0.4735079207476136, 3.003083688264095], # object pose values in radians
current_joint = [0.63,0.24,1.47,0.05,-0.89,3]) # current joint angles in radians
print(target_angle)



