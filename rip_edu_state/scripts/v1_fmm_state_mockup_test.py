#!/usr/bin/env python3

import rospy
from rip_edu_msgs.srv import (
    TextToSpeech, TextToSpeechResponse,
    SpeechToText, SpeechToTextResponse,
    HumanPoseDetection, HumanPoseDetectionResponse,
    NavigationToGoal, NavigationToGoalResponse,
    FreestyleMove, FreestyleMoveResponse,
    MarkHumanPositionToMap, MarkHumanPositionToMapResponse,
    Classification, ClassificationResponse,
    NavToGoalXYT, NavToGoalXYTResponse
)
from rip_edu_msgs.msg import *
import random

# Mock responses for services

def handle_text_to_speech(req):
    rospy.loginfo("TextToSpeech service called with text: %s", req.text)
    return TextToSpeechResponse(success=True)

def handle_speech_to_text(req):
    rospy.loginfo("SpeechToText service called")
    ans = random.randrange(0,2)
    if ans == 0:
        return SpeechToTextResponse(text="Yes", success=True)
    else:
        return SpeechToTextResponse(text="No", success=True)

def handle_human_pose_detection(req):
    rospy.loginfo("HumanPoseDetection service called")
    all_human_position = [
        [1.0, 2.0, 0.5],
        [3.0, 4.0, 0.1],
        [5.0, 6.0, -0.2]
    ]
    return HumanPoseDetectionResponse(all_human_position=all_human_position, success=True)

def handle_navigation_to_goal(req):
    rospy.loginfo("NavigationToGoal service called with pose_name: %s", req.pose_name)
    return NavigationToGoalResponse(success=True)

def handle_freestyle_move(req):
    rospy.loginfo("FreestyleMove service called with pose_name: %s", req.pose_name)
    return FreestyleMoveResponse(success=True)

def handle_mark_human_position_to_map(req):
    all_human_position = [
        [1.0, 2.0, 0.5],
        [3.0, 4.0, 0.1],
        [5.0, 6.0, -0.2]
    ]
    response = MarkHumanPositionToMapResponse()
    rospy.loginfo("MarkHumanPositionToMap service called with positions: %s", all_human_position)
    # return MarkHumanPositionToMapResponse(human_position_in_map=req.all_human_pose, success=True)
    
    # flattened_positions = [pos for sublist in all_human_position for pos in sublist]
    response.human_position_in_map = []
    for pos in all_human_position:
        hp = ListInListServiceResponse(number=pos)
        
        response.human_position_in_map.append(hp)
    response.success = True
    rospy.loginfo("test %s",response)
    return response

def handle_classification_detect(req):
    rospy.loginfo("Classification service called")
    list_of_class = ["Red", "Blue", "Kuai", "Table"]
    return ClassificationResponse(list_of_class=list_of_class, success=True)

def handle_nav_to_goal_xyt(req):
    rospy.loginfo(f"NavToGoalXYT service called with x: {req.x}, y: {req.y}, theta: {req.theta}")
    return NavToGoalXYTResponse(success=True)

def mock_service_server():
    rospy.init_node('mock_service_server')

    # Define services and their handlers
    rospy.Service('/rip/turtlebot/voice/text_to_speech', TextToSpeech, handle_text_to_speech)
    rospy.Service('/rip/turtlebot/voice/speech_to_text', SpeechToText, handle_speech_to_text)
    rospy.Service('/rip/turtlebot/vision/human_pose_detection', HumanPoseDetection, handle_human_pose_detection)
    rospy.Service('/rip/turtlebot/navigation/nav_to_goal_xyt', NavToGoalXYT, handle_nav_to_goal_xyt)
    rospy.Service('/rip/turtlebot/navigation/walk_fix', NavigationToGoal, handle_navigation_to_goal)
    rospy.Service('/rip/turtlebot/navigation/freestyle_move', FreestyleMove, handle_freestyle_move)
    rospy.Service('/rip/turtlebot/vision/mark_human_position_to_map', MarkHumanPositionToMap, handle_mark_human_position_to_map)
    rospy.Service('/rip/turtlebot/vision/classification_detect', Classification, handle_classification_detect)

    rospy.loginfo("Mock service server is ready")
    rospy.spin()

if __name__ == "__main__":
    mock_service_server()
