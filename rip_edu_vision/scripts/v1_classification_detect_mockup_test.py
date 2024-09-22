#!/usr/bin/env python3

import rospy
from rip_edu_msgs.srv import ClassificationDetectService, ClassificationDetectServiceRequest

def classification_detect_client():
    # Initialize the ROS node for the client
    rospy.init_node('classification_detect_client')

    # Wait for the service to be available
    rospy.wait_for_service('/rip/turtlebot/ai/classification_detect')

    try:
        # Create a service proxy to call the service
        classification_detect = rospy.ServiceProxy('/rip/turtlebot/ai/classification_detect', ClassificationDetectService)

        # Create an empty request object
        request = ClassificationDetectServiceRequest()

        # Call the service and get the response
        response = classification_detect(request)

        # Check the response
        if response.success:
            rospy.loginfo("Detected Classifications: %s", response.list_of_class)
        else:
            rospy.logwarn("Service call failed.")

        # Return the response for debugging purposes
        return response.list_of_class

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    # Call the client function
    detected_classes = classification_detect_client()

    # Print the detected classifications
    if detected_classes:
        print("Detected Classifications:")
        for classification in detected_classes:
            print(classification)
