#!/usr/bin/env python3

import rospy
from rip_edu_msgs.srv import HumanPoseDetection, HumanPoseDetectionRequest

def human_pose_detection_client():
    # Initialize the ROS node for the client
    rospy.init_node('human_pose_detection_client')

    # Wait for the service to be available
    rospy.wait_for_service('/rip/turtlebot/vision/humanposedetection')

    try:
        # Create a service proxy to call the service
        human_pose_detection = rospy.ServiceProxy('/rip/turtlebot/vision/humanposedetection', HumanPoseDetection)

        # Create an empty request object
        request = HumanPoseDetectionRequest()

        # Call the service and get the response
        response = human_pose_detection(request)

        # Check the response
        if response.success:
            rospy.loginfo("Detected Human Positions: %s", response.human_position)
        else:
            rospy.logwarn("Service call failed.")

        # Return the response for debugging purposes
        return response.human_position

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    # Call the client function
    detected_positions = human_pose_detection_client()

    # Print the detected human positions
    if detected_positions:
        print("Detected Human Positions in Map:")
        for position in detected_positions:
            print(position)
