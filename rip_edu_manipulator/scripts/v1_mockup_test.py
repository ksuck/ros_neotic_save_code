#!/usr/bin/env python3
import rospy
from rip_edu_msgs.srv import RipMoveGroup, RipMoveGroupRequest, RipMoveGroupResponse

class ManipulatorClient:
    def __init__(self):
        rospy.init_node('manipulator_client_node', anonymous=True)
        rospy.wait_for_service('/rip/turtlebot/manipulator/move_to_position')
        self.move_to_position_srv = rospy.ServiceProxy('/rip/turtlebot/manipulator/move_to_position', RipMoveGroup)

    def move_to_position_service(self, position, mode_kinematic, gripper):
        req = RipMoveGroupRequest()
        req.position = position
        req.mode_kinematic = mode_kinematic
        req.gripper = gripper

        try:
            res = self.move_to_position_srv(req)
            if res.success:
                rospy.loginfo(f"Successfully moved to position: {position}")
            else:
                rospy.logwarn(f"Failed to move to position: {position}")
            return res.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def run(self):
        # List of position commands
        commands = [
            ([90, 80, 100], 'FK', 1),
            ([90, 120, 150], 'FK', 1),
            ([90, 120, 150], 'FK', 0),
            ([90, 80, 120], 'FK', 0),
            ([90, 20, 40], 'FK', 0),
            ([90, 100, 60], 'FK', 0),
            ([90, 100, 60], 'FK', 1),
            ([90, 0, 0], 'FK', 0)
        ]

        for position, mode, grip in commands:
            success = self.move_to_position_service(position, mode, grip)
            if not success:
                rospy.logwarn("Stopping sequence due to failure.")
                break

if __name__ == '__main__':
    try:
        manipulator_client = ManipulatorClient()
        manipulator_client.run()
    except rospy.ROSInterruptException:
        pass
