#!/usr/bin/env python3

from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rip_edu_msgs.srv import MarkHumanPositionToMap, MarkHumanPositionToMapResponse, HumanPoseDetection, HumanPoseDetectionRequest
from rip_edu_msgs.msg import *
import cv2
from rospkg import RosPack
import numpy as np
import os
import traceback
from ultralytics import YOLO
import rosbag
import tf

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import tf.transformations as tft
import time
import math

pack_path = RosPack().get_path('rip_edu_vision')

class MarkHumanNode:
    def __init__(self, cam_color, cam_depth ,model_yolo = '', revers_cam = False, ros_rate = 500):
        #ชื่อ node
        rospy.init_node('MarkHumanPositionToMap', anonymous=True)
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
        self.listener = tf.TransformListener(cache_time=rospy.Duration(30))
        self.bag = rosbag.Bag('my_data.bag', 'w')  # Open rosbag in write mode
        
        
        self.service = rospy.Service('/rip/turtlebot/vision/mark_human_position_to_map', MarkHumanPositionToMap, self.mark_human_callback)
        rospy.loginfo("MarkHumanPositionToMap Service started!")

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



    def split_into_tuples(self, data, tuple_size):
        return tuple(tuple(data[i:i + tuple_size]) for i in range(0, len(data), tuple_size))

    def get_transform(self, listener, target_frame, source_frame):
        try:
            (trans, rot) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"Error: {e}")
            return None, None
        
    def mark_human_callback(self , req):
            dist = 0
            x = 0
            y = 0

            max_count = 0
            human_position_node = []
            new_tf_tran_map = []
            
            #1.57 , 3.14 ,
            rotate_pos = [1.57 , 3.14 , -1.57 , 0 ]
            try:
                while True:
                    if hasattr(self, 'frame_rgb') and hasattr(self, 'frame_depth') and hasattr(self, 'yaw') :
                        rospy.loginfo("loop")

                        try:
                            # size cam
                            h, w, _ = self.frame_rgb.shape

                            for i in range(4):
                            
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

                                            if dist < 1.625:
                                                x_camera = -(px1 - cx) * (dist / fh)  # Reverse the sign for correct direction
                                                y_camera = -(py1 - cy) * (dist / fv)  # Reverse the sign for correct direction
                                                #print(f'x: {x_camera} y: {y_camera} z: {dist}')


                                                x_world = x_camera + dist * math.cos(self.yaw)
                                                y_world = y_camera + dist * math.sin(self.yaw)

                                                human_position_node.append([float(self.yaw),float(x_world), float(y_world), float(dist)])
                                                cv2.imwrite(os.path.join(f'/home/naja/Home/src/rip_edu_vision/human_pose_detection{i}.jpg'),frame_)
                                                cv2.imwrite(os.path.join(pack_path,f'human_pose_detection{i}.jpg'),frame_copy)

                            break
                        except Exception as e:
                            tb = traceback.format_exc()
                            print(f"Traceback details:\n{tb}")
                            rospy.logerr(e)


                #check update 
                new_tf_tran_map = list(human_position_node)
                if human_position_node:
                    while not rospy.is_shutdown():
                    
                        for i, data in enumerate(human_position_node) :
                            #max_count = max_count + 1

                            print(f'all_human : {human_position_node}')

                            self.br.sendTransform(
                                (0, 0, 0),  # Translation (x, y, z)
                                tf.transformations.quaternion_from_euler(0, 0, 0),  # Orientation (roll, pitch, yaw)
                                rospy.Time.now(),
                                "map",  # Child frame
                                "world"  # Parent frame
                            )

                            self.br.sendTransform(
                                (float(data[1]) ,float(data[2]), float(data[3])),  # Translation (x, y, z)
                                tf.transformations.quaternion_from_euler(0, 0, float(data[0])), 
                                rospy.Time.now(),
                                f"human{i}",  # Child frame
                                "base_link"  # Parent frame
                            )




                            try:
                                self.listener.waitForTransform('world', f'human{i}', rospy.Time(), rospy.Duration(4.0))
                                trans, rot = self.get_transform(self.listener, 'map', f'human{i}')

                                if trans:
                                    rospy.loginfo(f"Transform from human{i} to map: x={trans[0]}, y={trans[1]}, z={trans[2]}")
                                    new_tf_tran_map[i] = [float(trans[0]), float(trans[1]), float(trans[2])]

                                if rot:
                                    rospy.loginfo(f"Rotation: x={rot[0]}, y={rot[1]}, z={rot[2]}, w={rot[3]}")
                            except Exception as e:
                                rospy.logwarn(f"Error getting transform for human{i}: {e}")

                        #test ค่า update
                        no_update = 0

                        for i, data in enumerate(new_tf_tran_map):
                            print(f'all_human : {human_position_node}')
                            print(f'tran : {new_tf_tran_map}')
                            if new_tf_tran_map[i][1] == human_position_node[i][1]:
                                no_update = no_update + 1

                        #print(no_update)


                        if no_update == 0:
                            print(f'all_human : {human_position_node}')
                            print(f'tran : {new_tf_tran_map}')

                            response = MarkHumanPositionToMapResponse()
                            response.human_position_in_map = []
                            for pos in new_tf_tran_map:
                                hp = ListInListServiceResponse(number=pos)

                                response.human_position_in_map.append(hp)
                            response.success = True
                            rospy.loginfo("test %s",response)
                            return response


                        self.rate.sleep()
                    else:
                        print(f'all_human : {human_position_node}')
                     
                        response = MarkHumanPositionToMapResponse()
                        response.human_position_in_map = []
                        response.success = True
                        rospy.loginfo("test %s",response)
                        return response

            
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
        MarkHumanNode(topic_camera_color, topic_camera_depth, yolov8_pose ,False)
        rospy.spin()
    except rospy.ROSInternalException:
        pass


if __name__ == '__main__':
    main()










