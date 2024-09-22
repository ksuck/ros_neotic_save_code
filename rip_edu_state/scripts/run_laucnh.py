#!/usr/bin/env python3

import rospy
import subprocess
from rip_edu_msgs.srv import RunLaunch, RunLaunchResponse

class LaunchFileService:
    def __init__(self):
        self.service = rospy.Service('/rip/turtlebot/state/run_launch_file', RunLaunch, self.run_launch_file)

    def run_launch_file(self, req):
        launch_file_path = req.path

        subprocess.Popen(['roslaunch', launch_file_path])
        res = RunLaunchResponse()
        res.success = True

        return res

def main():
    rospy.init_node('launch_file_service')
    service = LaunchFileService()
    rospy.loginfo("Launch file service is ready.")
    rospy.spin()

if __name__ == '__main__':
    main()