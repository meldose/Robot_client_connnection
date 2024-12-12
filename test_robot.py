from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(0.5)
target_pose=[-467, -345, 208, 0.01983048297176661, -0.40805741106893023, 0.9124022983282218, -0.02485854424955123]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)