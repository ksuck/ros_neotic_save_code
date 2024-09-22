#!/usr/bin/env python3

from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rip_edu_msgs.srv import EmptySeatDetection, EmptySeatDetectionResponse
import cv2
from rospkg import RosPack
import numpy as np
import os
import traceback
from ultralytics import YOLO
import tf
import geometry_msgs.msg
import tf.transformations as tft
import math

pack_path = RosPack().get_path('rip_edu_vision')

class empty_seat_node:
    def __init__(self, cam_color, cam_depth, model_yolo = '' ,conf = 0.5, revers_cam = False, ros_rate = 500):
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

        self.service = rospy.Service('/rip/turtlebot/vision/empty_seat_detection', EmptySeatDetection, self.EmptySeat)
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

    def EmptySeat(self , req):
        self.sucess = False
        frame_ = None

        item = []
        count_distance = 9999999
        threshold = 100

        total = ''

        while not rospy.is_shutdown():
            if hasattr(self, 'frame_rgb') and hasattr(self, 'frame_depth'):
            
                rospy.loginfo("loop")
                try:
                    frame_copy = self.frame_rgb.copy()
                    results = self.model.track(self.frame_rgb , persist=False, verbose=False)#จำกัดจำนวน [0]

                    # size cam
                    h, w, _ = self.frame_rgb.shape

                    center_pointx = w/2
                    center_pointy = h/2

                    #annotated_frame = results[0].plot()

                    #cv2.imshow("Tracking Results", annotated_frame)
                    
                    if results[0].boxes.id is not None:
                        #class
                        class_indices = results[0].boxes.cls.cpu().numpy()  # ดึงดัชนีคลาสที่ตรวจจับได้
                    
                        #id
                        track_ids = results[0].boxes.id.int().cpu()

                        

                        #box
                        boxes = results[0].boxes.xyxy.cpu()
                        frame_ = results[0].plot()

                        for box , id  , class_chair in zip(boxes,track_ids , class_indices):
                            if int(class_chair) == 0:
                                cv2.imwrite(os.path.join(f'/home/naja/Home/src/rip_edu_vision/chair_empty{id}.jpg'),frame_)
                                xmin, ymin, xmax, ymax = box

                                xmin = float(xmin)
                                ymin = float(ymin)
                                xmax = float(xmax)
                                ymax = float(ymax)

                                # Calculate the center point of the bounding box
                                pointx = round(xmin + (xmax - xmin) / 2)
                                pointy = round(ymin + (ymax - ymin) / 2)

                                #item.append((pointx,pointy))

                                px = center_pointx - pointx
                                py = center_pointy - pointy

                                px = pow(px,2)
                                py = pow(py,2)

                                distance = math.sqrt(px + py)

                                if distance < count_distance:
                                    count_distance = distance

                            
                            
                    

                    #cv2.imshow("Tracking Results", frame_ )


                    if pointx < center_pointx - 50:
                        total = 'seat_left'

                    elif pointx > center_pointx + 150:
                        total = 'seat_right'

                    else:
                        total = 'seat_center'
                    
                    if count_distance < 999999999:
                        respone = EmptySeatDetectionResponse()
                        respone.empty_seat = total
                        respone.success = True

                        return respone
                    else:
                        respone = EmptySeatDetectionResponse()
                        respone.empty_seat = total
                        respone.success = False

                        return respone

                    cv2.waitKey(1)
                    self.rate.sleep()
                except Exception as e:
                    tb = traceback.format_exc()
                    print(f"Traceback details:\n{tb}")
                    rospy.logerr(e)



    def run(self):
        print("start")
        s = rospy.Service('/rip/turtlebot/vision/empty_seat_detection', EmptySeatDetection , self.main_loop)
        rospy.spin()

def main():
    try:
        #ที่อยู่ topic กล้อง
        topic_camera_color = '/camera/rgb/image_color'
        topic_camera_depth = '/camera/depth/image_raw'
    
        #ที่อยู่ url Ai
        yolov8 = pack_path + f'/data/yolo_model/chair/CHAIR.pt'
        conf = 0.5
        empty_seat_node(topic_camera_color, topic_camera_depth, yolov8, conf ,False)
        rospy.spin()
    except rospy.ROSInternalException:
        pass


if __name__ == '__main__':
    main()