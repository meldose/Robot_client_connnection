
########## TESTING SCRIPT ######################################

from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(1)
target_pose=[0.6137146582584144, -0.6505111071166595, -1.25987692475429, 3.00390659125205, 1.1890477739172027, -0.7512713266246721]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)