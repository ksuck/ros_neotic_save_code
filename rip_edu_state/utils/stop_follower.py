#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import Joy
import subprocess
from rospkg import RosPack

pack_path = RosPack().get_path('turtlebot_follower')

class StopFollowerService(object):
	def __init__(self):
		self.pub = rospy.Publisher('/joy', Joy, queue_size=10)
		self.launch_file_path = pack_path + '/launch/follower.launch'

	def service_callback(self, req):
		try:
            # Assuming you want to kill the `launch` file process, 
            # this example uses pkill to stop a launch process with a specific name.
			# self.launch_file_path = req.path
			subprocess.run(['pkill', '-f', self.launch_file_path], check=True)
			return TriggerResponse(success=True)
		except subprocess.CalledProcessError as e:
			rospy.logerr("Failed to stop launch file: %s", str(e))
			return TriggerResponse(success=False)

	def run(self):
		rospy.init_node('stop_follower')
		rospy.Service('/rip/turtlebot/state/stop_follower', Trigger, self.service_callback)
		rospy.loginfo("Stop Follower is ready.")
		rospy.spin()

if __name__ == '__main__':
	try:
		stop_follower = StopFollowerService()
		stop_follower.run()
	except rospy.ROSInterruptException:
		pass
