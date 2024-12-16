
########## TESTING SCRIPT ######################################

from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(1)
target_pose=[0.5796684164073208, -0.6388726459181183, -1.205714038773689, 3.008671529308661, 1.2498721323881923, -0.7949765975549703]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)