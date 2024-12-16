
########## TESTING SCRIPT ######################################

from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(1)
target_pose=[0.6595518468955589, -0.6409620922778896, -1.1950431196670293, 3.00573849727458, 1.2686986222258927, -0.7160202503483175]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)