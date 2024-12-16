
########## TESTING SCRIPT ######################################

from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(1)
target_pose=[0.7164901253626474, -0.39697749196592286, -1.5542570281306336, 2.9979887360837667, 1.1620972775783767, -0.6418739425203702]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)