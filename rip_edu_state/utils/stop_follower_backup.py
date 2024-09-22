#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import Joy

class StopFollowerService(object):
	def __init__(self):
		self.pub = rospy.Publisher('/joy', Joy, queue_size=10)

	def service_callback(self, req):
		joy_msg = Joy()
		joy_msg.buttons = [0, 0, 0, 0, 0, 1]
		self.pub.publish(joy_msg)
		return TriggerResponse(success=True, message="Follower stopped")

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
