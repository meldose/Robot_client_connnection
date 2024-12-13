from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(0.5)
target_pose=[0.6257191984143152, -0.6410505429250599, -1.3423672773036655, 3.0083211347626935, 1.05453882492843, -1.5605939227703114]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)