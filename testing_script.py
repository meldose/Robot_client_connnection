#Initially transfer the object poses [X,Y;Z from the python code]

#conisder the roll, pitch and yaw values as : (-3.02,-0.06,1.41)

#Convert the cartesian to joint :

from neurapy.robot import Robot
r = Robot()
# target_pose = r.ik_fk("fk", target_angle = [0.2,0.2,0.2,0.2,0.2,0.2])

target_angle = r.ik_fk("ik", target_pose =[-0.524, -0.352, 0.208, -3.02,-0.06,1.41], # object pose values in radians
current_joint = [0.63,0.35,-1.34,3.01,1.15,-0.76]) # current joint angles in radians
print(target_angle)

# give the target pose :

from neurapy.robot import Robot

r = Robot()
r.set_mode("Automatic")
r.set_override(1)
target_pose=[0.6137146582584144, -0.6505111071166595, -1.25987692475429, 3.00390659125205, 1.1890477739172027, -0.7512713266246721]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)

