import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown

class MoveRobotToPose(Node):
    def __init__(self):
        super().__init__('move_robot_to_pose')
        roscpp_initialize(sys.argv)
        self.move_group = MoveGroupCommander("manipulator")  # Replace with your group name

    def move_to_pose(self, target_pose):
        self.move_group.set_pose_target(target_pose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return plan

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotToPose()

    # Define the target pose
    target_pose = Pose()
    target_pose.position.x = -0.519  # Replace with your target x
    target_pose.position.y = -0.362  # Replace with your target y
    target_pose.position.z =  0.055  # Replace with your target z
    target_pose.orientation.x = q_normalized[0]
    target_pose.orientation.y = q_normalized[1]
    target_pose.orientation.z = q_normalized[2]
    target_pose.orientation.w = q_normalized[3]

    # Move to the target pose
    success = node.move_to_pose(target_pose)
    if success:
        node.get_logger().info('Successfully moved to the target pose.')
    else:
        node.get_logger().error('Failed to move to the target pose.')

    # Shutdown
    roscpp_shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
