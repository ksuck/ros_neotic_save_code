#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from rip_edu_msgs.srv import EmptySeatDetectionResponse
from cv_bridge import CvBridge
from rospkg import RosPack
from std_srvs.srv import Empty
import cv2
import torch
import time

# COCO 128 class names (Sort in order same as COCO file)
COCO_CLASSES = [
    'empty_seat'
]
pack_path = RosPack().get_path('rip_edu_vision')

class DetectEmptySeatNode:
    def __init__(self, confidence_threshold):
        rospy.init_node('find_empty_seat', anonymous=True)
        self.bridge = CvBridge()
        self.confidence_threshold = confidence_threshold
        self.object_detected = False
        self.prev_time = time.time()
        self.detection_enabled = False

        # Load pretrained model
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=pack_path+'/data/yolo_model/chair/best.pt')
        # Subscribers and Publishers
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback, queue_size=1)
        self.image_pub = rospy.Publisher('result', Image, queue_size=1)
        self.cv_image = None

        # ROS Service for starting and stopping detection
        self.service_start_detection = rospy.Service('/rip/turtlebot/vision/find_empty_seat', Empty, self.start_detection)
        rospy.loginfo("DetectEmptySeat Service started!")

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def start_detection(self, req):
        res = EmptySeatDetectionResponse()

        while not rospy.is_shutdown():
            if self.cv_image is None:
                rospy.loginfo("Image is None")
                res.success=False
                res.empty_seat=""
                break

            cv_image = self.cv_image
            results = self.model(cv_image)
            object_detected = any(detection[4] >= self.confidence_threshold for detection in results.pred[0])

            # BB
            for detection in results.pred[0]:
                x1, y1, x2, y2 = map(int, detection[:4])
                cls = int(detection[5])
                conf = detection[4]

                if conf >= self.confidence_threshold:
                    # Draw
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (255, 0, 0), 2)

                    # Add text
                    text = f'{COCO_CLASSES[cls]}: {conf:.2f}'
                    cv2.putText(cv_image, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Calculate FPS
            curr_time = time.time()
            fps = 1 / (curr_time - self.prev_time)
            self.prev_time = curr_time

            # Display
            cv2.putText(cv_image, f'FPS: {fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Convert OpenCV image back to ROS image message
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            cv2.imwrite(pack_path + f'/img_debugged/detect_empty_seat.jpg', cv_image)
            self.image_pub.publish(image_msg)

            if object_detected == True:
                res.success=True
                res.empty_seat="chair"
                break

        return res

def main():
    try:
        confidence_threshold = 0.7
        DetectEmptySeatNode(confidence_threshold)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()