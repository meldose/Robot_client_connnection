
# from neurapy.robot import Robot # import the robot class
# r = Robot() # create an instance of the robot class
# target_angle = r.ik_fk("ik", target_pose =[-0.455, -0.447, 0.065, 0.07770435578717998, -3.1345624628521964, 0.9698778755924939],
# current_joint=[0.4129863573167266, -0.040367615708926226, -1.603394842405772,-0.07106004628138933, -1.5406368975914917, -2.2229109243340384])

# print(target_angle) # print the target joint angles



from neurapy.robot import Robot
r = Robot()
target_end_effector_pose = [-0.431, -0.467, 0.067, 0.0733116517009277, 3.138682979903034, 0.8232587543719854]
reference_joint_angles = [0.4129863573167266, -0.040367615708926226, -1.603394842405772,-0.07106004628138933, -1.5406368975914917, -2.2229109243340384]
joint_angle_solution = r.compute_inverse_kinematics(target_end_effector_pose, reference_joint_angles)

print(joint_angle_solution)



# Target Joint Angles: [0.787685213176135, -0.6126731905178867, -1.028139530057965, 0.07415923919141629, -1.5077299394348638, -1.6119271641085733]