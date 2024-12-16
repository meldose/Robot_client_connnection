
########## TESTING SCRIPT ######################################

from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(1)
target_pose=[0.7778648258274724, -0.5240964011625662, -1.3626536568546481, 3.000183864782744, 1.2341187552705948, -0.590389002752257]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)