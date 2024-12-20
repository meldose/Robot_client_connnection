
########## TESTING SCRIPT ######################################

from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(1)
target_pose=[0.6651033213830178, -0.7296441994676737, -1.066313453034224, 0.01580782830793816, -1.350361744827884, -0.22239712185590813]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)