
########## TESTING SCRIPT ######################################

from neurapy.robot import Robot
import time

r = Robot()
# r.gripper("on")
# time.sleep(0.2)
r.set_mode("Automatic")
r.set_override(0.4)
target_pose=[0.68481949781846, -0.6321197774300908, -0.848276904538431, 3.0107798218702837, 1.6245106608034727, -0.7382670573384716]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)
# r.gripper("off")