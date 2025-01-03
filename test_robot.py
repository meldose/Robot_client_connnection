
########## TESTING SCRIPT ######################################

from neurapy.robot import Robot
import time

r = Robot()

r.set_mode("Automatic")
r.set_override(0.7)
r.gripper("on")
target_pose=[0.7192742243584301, -0.5876637224953866, -1.0873323798202241, 3.0085883961377955, 1.4361029088726027, -2.2493009438456437]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)
r.gripper("off")
