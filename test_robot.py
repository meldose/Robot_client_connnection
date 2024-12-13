from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(0.5)
target_pose=[-0.518, -0.342, 0.089, 0.0005633121735972125, 0.707388044876899, -0.7068249569936026, -0.0005628637715330792]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)