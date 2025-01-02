
########## TESTING SCRIPT ######################################

from neurapy.robot import Robot
import time

r = Robot()
# r.gripper("on")
# time.sleep(0.2)
r.set_mode("Automatic")
r.set_override(0.4)
target_pose=[0.6848195750140478, -0.66930917673422, -1.1505391355276915, 3.0055338072503153, 1.288018423694767, -0.6930225092549912]
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)
# r.gripper("off")