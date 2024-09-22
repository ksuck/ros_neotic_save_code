#!/usr/bin/env python3

from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rip_edu_msgs.srv import DetectHumanInFront, DetectHumanInFrontResponse
import cv2
from rospkg import RosPack
import numpy as np
import os
import traceback
from ultralytics import YOLO
import tf
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import tf.transformations as tft
import time
import math

pack_path = RosPack().get_path('rip_edu_vision')


class detect_human_in_front:
    def __init__(self, cam_color, cam_depth, model_yolo = '' , revers_cam = False, ros_rate = 500):
        #ชื่อ node
        rospy.init_node('detect_human_in_front', anonymous=True)
        rospy.Subscriber(cam_color, Image, self.rgb_callback)
        rospy.Subscriber(cam_depth, Image, self.depth_callback)
        
        self.bridge = CvBridge()

        #กลับด้านกล้อง
        self.debug_revers = revers_cam

        #ความเร็วการทำงานกล้อง
        self.rate = rospy.Rate(ros_rate)

        self.model = YOLO(model_yolo)

        #TF Broadcaster
        self.br = tf.TransformBroadcaster()
        self.service = rospy.Service('/rip/turtlebot/vision/detect_human_in_front', DetectHumanInFront, self.detect_human_in_front_callback)
        rospy.loginfo("detect_human_in_front Service started!")

    def rgb_callback(self, data):
        try:
            frame_rgb = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.frame_rgb = frame_rgb
            if self.debug_revers == True:
                self.frame_rgb = cv2.flip(self.frame_rgb, 1)

        except :
            pass

    def depth_callback(self, data):
        try:
            frame_depth = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            self.frame_depth = frame_depth
            if self.debug_revers == True:
                self.frame_depth = cv2.flip(self.frame_depth, 1)

        except: 
            pass
        
    def detect_human_in_front_callback(self , req):
        self.sucess = False
        frame_ = None


        #rotate

        while not rospy.is_shutdown():
            
            if hasattr(self, 'frame_rgb') and hasattr(self, 'frame_depth'):
            
                try:
                    # size cam
                    h, w, _ = self.frame_rgb.shape


                    frame_copy = self.frame_rgb.copy()
                    results = self.model.track(self.frame_rgb , persist=False, verbose=False)#จำกัดจำนวน [0]

                    if results[0].boxes.id is not None:
                        #id
                        track_ids = results[0].boxes.id.int().cpu()

                        #box
                        boxes = results[0].boxes.xyxy.cpu()

                        #keypoint ตำแหน่งกระดูก
                        result_keypoint = results[0].keypoints.xyn.cpu().numpy()

                        frame_ = results[0].plot()
                        #cv2.imshow("human",frame_)

                        for box , id  in zip(boxes,track_ids):
                        
                            xmin, ymin, xmax, ymax = box

                            xmin = float(xmin)
                            ymin = float(ymin)
                            xmax = float(xmax)
                            ymax = float(ymax)

                            # Calculate the center point of the bounding box
                            px1 = round(xmin + (xmax - xmin) / 2)
                            py1 = round(ymin + (ymax - ymin) / 2)

                            #print(f'{px1} : {py1}')

                            cv2.circle(frame_copy, (abs(int(px1)), abs(int(py1))), 5, (255, 0, 255), cv2.FILLED)

                            if 0 < px1 < w and 0 < py1 < h:  # Check bounds for px1 and py1
                                dist = self.frame_depth[abs(int(py1)), abs(int(px1))]  # Fix the order for depth access

                                # Horizontal field of view in degrees of camera
                                horizontal_fov = 57  # <---- from data sheet
                                vertical_fov = 43  # <---- from data sheet

                                fh = (w / 2) / np.tan(np.radians(horizontal_fov) / 2)
                                fv = (h / 2) / np.tan(np.radians(vertical_fov) / 2)

                                # Calculate principal point (in pixels)
                                cx = w / 2
                                cy = h / 2

                                dist = dist / 1000  # Convert mm to meters
                                #print(dist)

                                if dist < 0.1:
                                    self.sucess = True

                                    response = DetectHumanInFrontResponse()
                                    response.success = True
                                    return response
                                    
                    else:
                        response = DetectHumanInFrontResponse()
                        response.success = False
                        return response
                                    


                    cv2.destroyAllWindows()
                
                    

                    cv2.waitKey(1)
                    self.rate.sleep()
                except Exception as e:
                    tb = traceback.format_exc()
                    print(f"Traceback details:\n{tb}")
                    rospy.logerr(e)
            

def main():
    try:
        #ที่อยู่ topic กล้อง
        topic_camera_color = '/camera/rgb/image_color'
        topic_camera_depth = '/camera/depth/image_raw'
    
        #ที่อยู่ url Ai
        yolov8_pose = pack_path + f'/data/yolo_model/hand/yolov8n-pose.pt'
        detect_human_in_front(topic_camera_color, topic_camera_depth, yolov8_pose , False)
        rospy.spin()
    except rospy.ROSInternalException:
        pass


if __name__ == '__main__':
    main()










