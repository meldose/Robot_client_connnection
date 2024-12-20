
########## TESTING SCRIPT ######################################

from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(1)
target_pose=[0.6610732227076228, -0.897886163639536, -0.788282902025545, 3.010315865326606, 1.4174037786275882, -0.7349648516122518]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)