
########## TESTING SCRIPT ######################################

from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(1)
target_pose=[0.702843160168057, -0.8945068496669094, -0.7796926487661704, 3.0091058682218677, 1.434730360689965, -0.694960850522939]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)