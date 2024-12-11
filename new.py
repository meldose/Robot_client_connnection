from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(1)
target_pose=[-3,1.48,0.26,-0.0002,1.38,1.70]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)