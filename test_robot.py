
########## TESTING SCRIPT ######################################

from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(1)
target_pose=[0.7158133433124352, -0.48454564330280564, -1.4305499175182936, 3.000122417784659, 1.1977798019490207, -0.6481034956353517]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)