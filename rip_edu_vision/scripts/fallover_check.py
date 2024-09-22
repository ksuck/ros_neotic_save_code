#!/usr/bin/env python3

from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rip_edu_msgs.srv import EmptySeatDetection, EmptySeatDetectionResponse
from rospkg import RosPack
import cv2

import numpy as np
import os
import traceback
from ultralytics import YOLO
import tf
import geometry_msgs.msg
import tf.transformations as tft
import math

#pack_path = RosPack().get_path('rip_edu_vision')

class failover_node:
    def __init__(self, cam_color,cam_depth,model_yolo = '' ,conf = 0.5, revers_cam = False, ros_rate = 500):
        #ชื่อ node
        rospy.init_node('empty_seat_detection', anonymous=True)
        rospy.Subscriber(cam_color, Image, self.rgb_callback)
        rospy.Subscriber(cam_depth, Image, self.depth_callback)
        self.bridge = CvBridge()

        #กลับด้านกล้อง
        self.debug_revers = revers_cam

        #ความเร็วการทำงานกล้อง
        self.rate = rospy.Rate(ros_rate)

        self.model = YOLO(model_yolo)
        self.conf = conf

        #TF Broadcaster
        self.br = tf.TransformBroadcaster()    

        self.service = rospy.Service( '/rip/turtlebot/vision/empty_seat_detection', EmptySeatDetection, self.fallover_main)
        rospy.loginfo("empty_seat Service started!")

    

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

    def fallover_main(self , req):
        self.sucess = False
        frame_ = None

        while True:
            if  hasattr(self, 'frame_rgb') and hasattr(self, 'frame_depth'):
            
                try:
                    frame_copy = self.frame_rgb.copy()
                    

                    # size cam
                    h, w, _ = self.frame_rgb.shape

                    high_haft = int(h / 2)

                    cat_frame = self.frame_rgb[high_haft : h, 0 : w]

                    center_pointx = w/2
                    center_pointy = h/2

                    
               
                   
                    results = self.model.track(cat_frame , persist=False, verbose=False)#จำกัดจำนวน [0]
                    frame_res = results[0].plot()

                    if results[0].boxes.id is not None:
                        #class
                        class_indices = results[0].boxes.cls.cpu().numpy()  # ดึงดัชนีคลาสที่ตรวจจับได้

                        #id
                        track_ids = results[0].boxes.id.int().cpu()

                        #keypoint ตำแหน่งกระดูก
                        # result_keypoint = results[0].keypoints.xyn.cpu().numpy()

                        #box
                        boxes = results[0].boxes.xyxy.cpu()

                        #frane show
                        frame_ = results[0].plot()

                        confidence_scores = results[0].boxes.conf.cpu().numpy()

                        for box , id  , cls , conf_fallover in zip(boxes,track_ids ,  class_indices , confidence_scores):
                            xmin, ymin, xmax, ymax = box

                            xmin = float(xmin)
                            ymin = float(ymin)
                            xmax = float(xmax)
                            ymax = float(ymax)

                            
                            
                            if int(cls) == 0 and conf_fallover > self.conf:
                           
                                    respone = EmptySeatDetectionResponse()
                                    respone.empty_seat = 'injured'
                                    respone.success = True
                                    print("injured")
                                    return respone


                    else:
                        respone = EmptySeatDetectionResponse()
                        respone.empty_seat = ''
                        respone.success = True

                        return respone



                    cv2.imshow("Tracking Results", frame_res)
                    cv2.waitKey(1)
                    self.rate.sleep()

                except Exception as e:
                    tb = traceback.format_exc()
                    print(f"Traceback details:\n{tb}")

                    # faill
                    respone = EmptySeatDetectionResponse()
                    respone.empty_seat = ''
                    respone.success = False

                    return respone
def main():
    try:
        #ที่อยู่ topic กล้อง
        topic_camera_color = '/camera/rgb/image_color'
        topic_camera_depth = '/camera/depth/image_raw'
    
        #ที่อยู่ url Ai
        yolov8_pose = f'/home/naja/Home/src/rip_edu_vision/scripts/fallover2.pt' 
        conf = 0.5
        failover_node(topic_camera_color,topic_camera_depth,yolov8_pose, conf ,False)
        rospy.spin()
    except rospy.ROSInternalException:
        pass


if __name__ == '__main__':
    main()