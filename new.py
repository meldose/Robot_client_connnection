from neurapy.robot import Robot

r = Robot()
target_pose=[-0.785,-0.1,-0.004,180,0,-90]
linear_property = {
    "target_joint": target_pose,
    "speed": 0.25,
    "acceleration": 0.1,
    "jerk": 100,
    "rotation_speed": 1.57,
    "rotation_acceleration": 5.0,
    "rotation_jerk": 100,
    "blending": True,
    "blending_mode": 2,
    "blend_radius": 0.01,
}
r.move_joint(**linear_property)