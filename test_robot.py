
########## TESTING SCRIPT ######################################

from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(0.5)
target_pose=[0.6137147266880328, -0.6251236948653252, -1.22462669038971, 3.0069279288899766, 1.2491246005338323, -0.7600382016493035]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)