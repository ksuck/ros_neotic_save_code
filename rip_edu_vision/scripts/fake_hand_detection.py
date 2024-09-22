#!/usr/bin/env python3

import rospy
from rip_edu_msgs.srv import HandDetection, HandDetectionResponse

def hand_detection_callback(request):
    # For the sake of the example, the server will always return "left"
    rospy.loginfo("Hand detection request received.")
    
    # Logic for detecting hand direction
    direction = "left"
    success = True
    
    rospy.loginfo(f"Hand direction: {direction}, success: {success}")
    
    return HandDetectionResponse(direction=direction, success=success)

def hand_detection_server():
    rospy.init_node('hand_detection_server')
    
    # Define the service
    service = rospy.Service('/rip/turtlebot/vision/hand_detection', HandDetection, hand_detection_callback)
    
    rospy.loginfo("HandDetection Service started!")
    
    # Keep the service running
    rospy.spin()

if __name__ == "__main__":
    try:
        hand_detection_server()
    except rospy.ROSInterruptException:
        pass
