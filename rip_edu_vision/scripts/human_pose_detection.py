#!/usr/bin/env python3

from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rip_edu_msgs.msg import *
from rip_edu_msgs.srv import *
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

class human_pose_detection:
    def __init__(self, cam_color, cam_depth, model_yolo = '' , revers_cam = False, ros_rate = 500):
        #ชื่อ node
        rospy.init_node('human_pose_detection', anonymous=True)
        rospy.Subscriber(cam_color, Image, self.rgb_callback)
        rospy.Subscriber(cam_depth, Image, self.depth_callback)
        rospy.Subscriber('/odom', Odometry, self.callback)

        self.current_yaw = 0.0
        self.start_yaw = 0.0

        self.init_yaw = 0.0
        
        
        self.bridge = CvBridge()

        #กลับด้านกล้อง
        self.debug_revers = revers_cam

        #ความเร็วการทำงานกล้อง
        self.rate = rospy.Rate(ros_rate)

        self.model = YOLO(model_yolo)

        #TF Broadcaster
        self.br = tf.TransformBroadcaster()
        self.service = rospy.Service('/rip/turtlebot/vision/humanposedetection', HumanPoseDetection, self.hand_detection_callback)
        rospy.loginfo("Humanpose Service started!")

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

    def callback(self, data):
        try:
            orientation = data.pose.pose.orientation
            self.roll, self.pitch, self.yaw = tft.euler_from_quaternion([
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w
            ])

            #rospy.loginfo("Roll: %f, Pitch: %f, Yaw: %f", self.roll, self.pitch, self.yaw)
        except: 
            pass
    
    
    def rotate_to(self, angular) :
        twist = Twist()
        pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -0.5 if angular < 0 else 0.5

        while round(self.yaw, 2) != round(angular, 2) :
            pub.publish(twist)
            self.rate.sleep()
        twist.angular.z = 0
        pub.publish(twist)
        rospy.sleep(1) 

    
    '''

    def rotate(self,angle_radians):
        twist = Twist()
        pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0.5 if angle_radians > 0 else -0.5  # Set rotation speed and direction

        self.start_yaw = self.yaw
        target_yaw = self.start_yaw + angle_radians
        rospy.loginfo(f"Starting rotation from {self.start_yaw} to {target_yaw} radians...")
        
        while not rospy.is_shutdown():
            pub.publish(twist)
            
            # Normalize yaw to be within the range [0, 2*pi)
            normalized_yaw = self.yaw % (2 * np.pi)

            # Check if we have reached the target yaw
            if (angle_radians > 0 and normalized_yaw >= target_yaw) or (angle_radians < 0 and normalized_yaw <= target_yaw):
                rospy.loginfo("Rotation completed.")
                break
            
            self.rate.sleep()
        
        twist.angular.z = 0
        pub.publish(twist)
        rospy.sleep(1) 
        '''
    
    def hand_detection_callback(self , req):
        self.sucess = False
        frame_ = None

        human_position_node = []

        #rotate 1.57 , 3.14 , -1.57 
        rotate_pos = [0 , -1.57 ]

        while not rospy.is_shutdown():
            
            if hasattr(self, 'frame_rgb') and hasattr(self, 'frame_depth') and hasattr(self, 'yaw') :
            
                rospy.loginfo("loop")
                
                try:
                    # size cam
                    h, w, _ = self.frame_rgb.shape

                    for i in range(2):
                    
                        print(i)

                        print("rotation")
                        self.rotate_to(rotate_pos[i])
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

                                    if dist > 0.5:
                                        x_camera = -(px1 - cx) * (dist / fh)  # Reverse the sign for correct direction
                                        y_camera = -(py1 - cy) * (dist / fv)  # Reverse the sign for correct direction
                                        #print(f'x: {x_camera} y: {y_camera} z: {dist}')


                                        x_world = x_camera + dist * math.cos(self.yaw)
                                        y_world = y_camera + dist * math.sin(self.yaw)

                                        human_position_node.append([float(self.yaw),float(x_world), float(y_world), float(dist)])
                                        cv2.imwrite(os.path.join(f'/home/naja/Home/src/rip_edu_vision/human_pose_detection{i}.jpg'),frame_)
                                        cv2.imwrite(os.path.join(pack_path,f'human_pose_detection{i}.jpg'),frame_copy)

                                
                    
                    print(f'all_human : {human_position_node}')
                    print(f'Sucess    : {self.sucess}')

                    print(type(human_position_node))
                    print(type(human_position_node[0]))
                    print(type(human_position_node[0][1]))
                    self.sucess = True

                    
                
                    rospy.loginfo(f'all_human : {human_position_node}')
                    rospy.loginfo(f'Sucess    : {self.sucess}')

                    cv2.destroyAllWindows()



                    response = HumanPoseDetectionResponse()
                    response.human_position = []
                    for pos in human_position_node:
                        hp = ListInListServiceResponse(number=pos)
                        response.human_position(hp)
                    response.success = True 
                
                    return response

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
        human_pose_detection(topic_camera_color, topic_camera_depth, yolov8_pose , False)
        rospy.spin()
    except rospy.ROSInternalException:
        pass


if __name__ == '__main__':
    main()










