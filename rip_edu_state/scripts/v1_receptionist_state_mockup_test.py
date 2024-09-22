#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
from rip_edu_msgs.srv import TextToSpeech, TextToSpeechResponse
from rip_edu_msgs.srv import SpeechToText, SpeechToTextResponse
from rip_edu_msgs.srv import DetectHumanInFront, DetectHumanInFrontResponse
from rip_edu_msgs.srv import EmptySeatDetection, EmptySeatDetectionResponse
from rip_edu_msgs.srv import NavigationToGoal, NavigationToGoalResponse
global count
count = 1
global count_seat
count_seat = 1
def text_to_speech_callback(req):
    rospy.loginfo("TextToSpeech service called with text: %s", req.text)
    return TextToSpeechResponse(success=True)

def speech_to_text_callback(req):
    global count
    if count == 1:
        rospy.loginfo("SpeechToText service called")
        
        # Simulate recognizing the name "John Doe"
        count = count+1
        return SpeechToTextResponse(success=True, text="julia")
    else:
        count = 0
        return SpeechToTextResponse(success=True, text="water")

def detect_human_in_front_callback(req):
    rospy.loginfo("DetectHumanInFront service called")
    return DetectHumanInFrontResponse(success=True)

def empty_seat_detection_callback(req):
    rospy.loginfo("EmptySeatDetectionService service called")
    global coucount_seatnt
    if count_seat == 1:
        rospy.loginfo("SpeechToText service called")
        
        # Simulate recognizing the name "John Doe"
        count_seat = count_seat + 1
        return EmptySeatDetectionResponse(success=True, empty_seat = "seat_right")
    else:
        count_seat = 0
        return EmptySeatDetectionResponse(success=True, empty_seat = "seat_left")

def navigation_to_goal_callback(req):
    rospy.loginfo("NavigationToGoal service called with goal: %s", req.pose_name)
    return NavigationToGoalResponse(success=True)

def main():
    rospy.init_node('mock_services_server')

    rospy.Service('/rip/turtlebot/voice/text_to_speech', TextToSpeech, text_to_speech_callback)
    rospy.Service('/rip/turtlebot/voice/speech_to_text', SpeechToText, speech_to_text_callback)
    rospy.Service('/rip/turtlebot/vision/detect_human_in_front', DetectHumanInFront, detect_human_in_front_callback)
    rospy.Service('/rip/turtlebot/vision/empty_seat_detection', EmptySeatDetection, empty_seat_detection_callback)
    rospy.Service('/rip/turtlebot/navigation/nav_to_goal_xyt', NavigationToGoal, navigation_to_goal_callback)
    rospy.Service('/rip/turtlebot/navigation/walk_fix', NavigationToGoal, navigation_to_goal_callback)

    rospy.loginfo("Mock test server ready.")
    rospy.spin()

if __name__ == '__main__':
    main()
