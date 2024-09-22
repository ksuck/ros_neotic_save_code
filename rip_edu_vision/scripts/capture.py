import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Initialize the CvBridge class
bridge = CvBridge()

# Directory to save captured images
output_dir = '/home/naja/Home/src/rip_edu_vision/scripts/captured_images'

# Create directory if it doesn't exist
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Counter to name saved images uniquely
image_counter = 0

def image_callback(msg):
    global image_counter
    
    try:
        # Convert the ROS Image message to a format OpenCV understands
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Display the image using OpenCV
        cv2.imshow("Camera Feed", cv_image)
        
        # Check for Space bar press
        key = cv2.waitKey(1)  # Wait for 1 ms
        if key == ord(' '):  # Space bar key code
            # Save the image
            image_filename = os.path.join(output_dir, f'image_{image_counter:04d}.jpg')
            cv2.imwrite(image_filename, cv_image)
            print(f"Image saved: {image_filename}")
            image_counter += 1

    except CvBridgeError as e:
        print(f"CvBridge Error: {e}")

def main():
    # Initialize the ROS node
    rospy.init_node('camera_listener', anonymous=True)
    
    # Subscribe to the camera topic
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    
    # Keep the node running
    rospy.spin()
    
    # Close any OpenCV windows when done
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
