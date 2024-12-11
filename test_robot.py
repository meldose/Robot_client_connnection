from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(0.5)
target_pose=[-3.008594728329113, 1.4812225184280459, 0.2678806778148677, -0.0002137209982971917, 1.390911899266658, 1.7030355307580345]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)