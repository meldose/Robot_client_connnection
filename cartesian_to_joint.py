
from neurapy.robot import Robot
r = Robot()
target_angle = r.ik_fk("ik", target_pose =[-0.52, -0.41, 0.234, 0.016077522999917958, 3.1403538053266784, 0.8840185473926665], # object pose values in radians
current_joint = [0.63,0.35,-1.34,3.01,1.15,-0.76]) # current joint angles in radians
print(target_angle)



