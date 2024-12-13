# #Initially transfer the object poses [X,Y;Z from the python code] 

# Whem adding the Z value there should be an offset of 0.12m in z direction

# #conisder the roll, pitch and yaw values as : (-3.02,-0.06,1.41)

# #Convert the cartesian to joint :

# from neurapy.robot import Robot
# r = Robot()
# # target_pose = r.ik_fk("fk", target_angle = [0.2,0.2,0.2,0.2,0.2,0.2])

# target_angle = r.ik_fk("ik", target_pose =[-0.524, -0.352, 0.208, -3.02,-0.06,1.41], # object pose values in radians
# current_joint = [0.63,0.35,-1.34,3.01,1.15,-0.76]) # current joint angles in radians
# print(target_angle)

# # give the target pose :

# from neurapy.robot import Robot

# r = Robot()
# r.set_mode("Automatic")
# r.set_override(1)
# target_pose=[0.6137146582584144, -0.6505111071166595, -1.25987692475429, 3.00390659125205, 1.1890477739172027, -0.7512713266246721]
# linear_property = {
#     "target_joint": target_pose,
# }
# r.move_joint(**linear_property)



from neurapy.robot import Robot

# Initialize the robot instance
r = Robot()

# Set robot mode to automatic and set override to 1 for full control
r.set_mode("Automatic")
r.set_override(1)

# Tramsfer the X;Y;Z value to the robotic Controller

# Whem adding the Z value there should be an offset of 0.12m in z direction (dont take the initial value from the camera)


# Define the target pose (Position: X, Y, Z and Orientation: roll, pitch, yaw)
target_pose = [-0.524, -0.352, 0.208, -3.02, -0.06, 1.41]  # [X, Y, Z, roll, pitch, yaw] in radians

# Define the current joint angles
current_joint = [0.63, 0.35, -1.34, 3.01, 1.15, -0.76]  # current joint angles in radians

# Perform inverse kinematics (IK) to find the target joint angles based on the current pose
target_angle = r.ik_fk("ik", target_pose=target_pose, current_joint=current_joint)

# Print the computed target joint angles
print("Calculated target joint angles:", target_angle)

# Once the IK is computed, set the robot's target joint angles
linear_property = {
    "target_joint": target_angle,
}

# Move the robot to the calculated joint positions
r.move_joint(**linear_property)

# Optional: You can add a confirmation message once the robot has moved.
print("Robot has moved to the target joint configuration.")
