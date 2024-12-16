
########## TESTING SCRIPT ######################################

from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(1)
target_pose=[0.6814509404687945, -0.728331173323554, -1.061101500332202, 3.0067721406368095, 1.3177131408710756, -0.7006484920967712]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)