
########## TESTING SCRIPT ######################################

from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(0.5)
target_pose=[0.6313211802161058, -0.6291259870389171, -1.2159404505433031, 3.0064153518738306, 1.2560433625325484, -0.7430311431863563]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)