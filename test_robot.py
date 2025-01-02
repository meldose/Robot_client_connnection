
########## TESTING SCRIPT ######################################

from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(1)
target_pose=[0.728257047134004, -1.101075949642333, -0.7295555335670018, 1.6836375623625968, 2.7432265644613842, -1.739486131005509]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)