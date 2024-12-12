from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(0.5)
target_pose=[-0.519, -0.362, 0.055, -2.058652790827435, -0.4735079207476136, 3.003083688264095]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)