
from neurapy.robot import Robot # import the robot class
r = Robot() # create an instance of the robot class
target_angle = r.ik_fk("ik", target_pose =[-0.495, -0.307, 0.055, 0.07744464551997962, -3.1226535425969555, 1.0911965632134155],
current_joint=[0.4129863573167266, -0.040367615708926226, -1.603394842405772,-0.07106004628138933, -1.5406368975914917, -2.2229109243340384])

print(target_angle) # print the target joint angles

