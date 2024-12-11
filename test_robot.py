from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(0.5)
target_pose=[-3.008594776381439, 1.3910851363612151, 0.08055689260676392, -0.00021092979941038508, 1.6683700291781138, 1.7029772275306843]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)