#!/usr/bin/env python3
import rospy
from rip_edu_msgs.srv import TextToSpeech, TextToSpeechResponse
from rip_edu_msgs.srv import SpeechToText, SpeechToTextResponse
from rip_edu_msgs.srv import HandDetection, HandDetectionResponse
from rip_edu_msgs.srv import FreestyleMove, FreestyleMoveResponse
from rip_edu_msgs.srv import RipMoveGroup, RipMoveGroupResponse
from rip_edu_msgs.srv import RunLaunch, RunLaunchResponse
from std_srvs.srv import Trigger, TriggerResponse

# Mock TextToSpeech service
def mock_text_to_speech(req):
    rospy.loginfo("Mock TextToSpeech request received: %s", req.text)
    return TextToSpeechResponse(success=True)

# Mock SpeechToText service
def mock_speech_to_text(req):
    rospy.loginfo("Mock SpeechToText request received")
    return SpeechToTextResponse(success=True, text="stop")

# Mock HandDetection service
def mock_hand_detection(req):
    rospy.loginfo("Mock HandDetection request received")
    return HandDetectionResponse(success=True, direction="right")

# Mock FreestyleMove service
def mock_freestyle_move(req):
    rospy.loginfo("Mock FreestyleMove request received: %s", req.pose_name)
    return FreestyleMoveResponse(success=True)

# Mock RipMoveGroup service
def mock_move_to_position(req):
    rospy.loginfo("Mock MoveToPosition request received")
    return RipMoveGroupResponse(success=True)

# Mock RunLaunch service
def mock_run_launch_file(req):
    rospy.loginfo("Mock RunLaunchFile request received")
    return RunLaunchResponse(success=True)

# Mock StopFollower (Trigger) service
def mock_stop_follower(req):
    rospy.loginfo("Mock StopFollower request received")
    return TriggerResponse(success=True, message="Stopped follower")

def main():
    rospy.init_node('mock_services')

    # Creating the mock service servers
    rospy.Service('/rip/turtlebot/voice/text_to_speech', TextToSpeech, mock_text_to_speech)
    rospy.Service('/rip/turtlebot/voice/speech_to_text', SpeechToText, mock_speech_to_text)
    rospy.Service('/rip/turtlebot/vision/hand_detection', HandDetection, mock_hand_detection)
    rospy.Service('/rip/turtlebot/navigation/walk_fix', FreestyleMove, mock_freestyle_move)
    rospy.Service('/rip/turtlebot/manipulator/move_to_position', RipMoveGroup, mock_move_to_position)
    rospy.Service('/rip/turtlebot/state/run_launch_file', RunLaunch, mock_run_launch_file)
    rospy.Service('/rip/turtlebot/state/stop_follower', Trigger, mock_stop_follower)

    rospy.loginfo("Mock services are ready")
    rospy.spin()

if __name__ == '__main__':
    main()
