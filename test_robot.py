
########## TESTING SCRIPT ######################################

from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(1)
target_pose=[0.6791590343278399, -0.8054708848746635, -0.9249594963788765, 3.008644267055404, 1.3758873825913724, -0.7110134383647276]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)