#!/usr/bin/env python3

import rospy
from rip_edu_msgs.srv import MarkHumanPositionToMap, MarkHumanPositionToMapRequest

def mark_human_position_to_map_client():
    # Initialize the ROS node for the client
    rospy.init_node('mark_human_position_to_map_client')

    # Wait for the service to be available
    rospy.wait_for_service('/rip/turtlebot/vision/MarkHumanPositionToMap')

    try:
        # Create a service proxy to call the service
        mark_human_position = rospy.ServiceProxy('/rip/turtlebot/vision/MarkHumanPositionToMap', MarkHumanPositionToMap)

        # Create a request object
        request = MarkHumanPositionToMapRequest()

        # Mock input data: list of human poses [x, y, theta]
        request.all_human_pose = [
            [10.0, 2.0, 0.1],
            [1.0, 1.0, 1.0],
            [1.5, 0.5, 0.2],
            [2.0, 1.0, 0.3],
            [20.5, 3.5, 0.4],
            [30.0, 4.0, 0.5],
            [3.5, 4.5, 0.6],
            [4.0, 5.0, 0.7],
            [4.5, 5.5, 0.8],
            [5.0, 6.0, 0.9],
            [5.5, 6.5, 1.0]
        ]

        # Call the service and get the response
        response = mark_human_position(request)

        # Check the response
        if response.success:
            rospy.loginfo("Mapped Human Positions: %s", response.human_position_in_map)
        else:
            rospy.logwarn("Service call failed.")

        # Return the response for debugging purposes
        return response.human_position_in_map

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    # Call the client function
    mapped_positions = mark_human_position_to_map_client()

    # Print the mapped human positions
    if mapped_positions:
        print("Mapped Human Positions:")
        for position in mapped_positions:
            print(position)
