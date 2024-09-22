#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from rip_edu_msgs.srv import RipMoveGroup, RipMoveGroupResponse
from dynamixel_sdk import *
import pyIK
import dynamixelControler as dyc
import time

class ManipulatorControl :
    def __init__(self) :
        self.synchrony_time = 3
        ik_met_list = ["simplyik", "moveit"]
        self.ik_met = ik_met_list[0]

        rospy.init_node('manipulator_node', anonymous=True)
        self.dy = dyc.dyControl('/dev/ttyUSB2') # <---- change this!!
        self.dy.move_synchrony_motors([90, 100, 105, 90], self.synchrony_time)
        rospy.Service('/rip/turtlebot/manipulator/move_to_position', RipMoveGroup, self.manipulatorControl_callback)

        if self.ik_met == "simplyik" :
            self.sim_ik = pyIK.simIK(titf_json_path="/home/naja/Home/src/rip_edu_manipulator/scripts/titf_json/titf1.json")

    def manipulatorControl_callback(self, req) :
        """
        mode list : (default to IK)
            'FK' : take 3 degree in position list and direcly move motor.
            'IK' : take 3 cordinate (x, y, z) in position list to IK and move motor.
        """
            
        position = req.position
        mode = req.mode_kinematic
        grip = req.gripper

        grip_deg = 150 if int(grip) == 1 else 90

        if mode == "FK" :
            j1, j2, j3 = list(position)
            move_to = [j1, j2 + 90, j3 + 90, grip_deg]
            self.dy.move_synchrony_motors(move_to, self.synchrony_time)
            time.sleep(self.synchrony_time + 1)
            rospy.loginfo("Succeeded to move.")
            return RipMoveGroupResponse(True)
        elif mode == "IK" :
            if self.ik_met == "simplyik" :
                try :
                    x, y, z = position
                    ikpos = [x, y, z + 100]
                    j1, j2, j3 = list(self.sim_ik.IK(ikpos))
                    joint_angles = [j1 + 90, -j2 + 90, j3 + 90, grip_deg]
                    self.dy.move_synchrony_motors(joint_angles, self.synchrony_time)
                    time.sleep(self.synchrony_time + 1)
                    rospy.loginfo("Succeeded to move.")
                    return RipMoveGroupResponse(True)
                except Exception as e :
                    rospy.logerr("Inverse kinematics failed with error: %s", e)
                    return RipMoveGroupResponse(False)
            elif self.ik_met == "moveit" :
                target_pose = PoseStamped()
                target_pose.header.frame_id = "base_link"  # Replace with actual frame ID
                target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z = list(position)
                rospy.wait_for_service('compute_ik')
                try :
                    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
                    ik_request = GetPositionIKRequest()
                    ik_request.ik_request.group_name = "arm" # Replace with actual Arm
                    ik_request.ik_request.pose_stamped = target_pose
                    ik_response = compute_ik(ik_request)

                    if ik_response.error_code.val == ik_response.error_code.SUCCESS :
                        # Get joint angles from IK solution
                        j1, j2, j3 = list(ik_response.solution.joint_state.position)
                        joint_angles = [j1, j2, j3, grip_deg]
                        self.dy.move_synchrony_motors(joint_angles, self.synchrony_time)
                        rospy.loginfo("Succeeded to move.")
                        return RipMoveGroupResponse(True)
                    else :
                        rospy.logerr("Inverse kinematics failed with error code: %d", ik_response.error_code.val)
                        return RipMoveGroupResponse(False)
                except rospy.ServiceException as e :
                    rospy.logerr("Service call failed: %s", e)
                    return RipMoveGroupResponse(False)
        else : 
            rospy.logerr("Mode not found")
            return RipMoveGroupResponse(False)

    def run(self) :
        rospy.spin()

if __name__ == '__main__' :
    try:
        manipulator_control = ManipulatorControl()
        manipulator_control.run()
    except rospy.ROSInterruptException :
        pass
