#!/usr/bin/env python3

from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rip_edu_msgs.srv import HumanPoseDetection, HumanPoseDetectionResponse
import cv2
from rospkg import RosPack
import numpy as np
import os
import traceback
from ultralytics import YOLO
import tf
import geometry_msgs.msg

pack_path = RosPack().get_path('rip_edu_vision')

class HandDetectionNode:
    def __init__(self, cam_color, cam_depth, model_yolo = '' , revers_cam = False, ros_rate = 500):
        #ชื่อ node
        rospy.init_node('human_pose_tf_detection', anonymous=True)
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

    def hand_detection_callback(self , req):
        self.sucess = False
        self.direction = 'None'
        near_id = 0
        frame_ = None

        while not rospy.is_shutdown():
            if hasattr(self, 'frame_rgb') and hasattr(self, 'frame_depth'):
                rospy.loginfo("loop")
                try:
                    frame_copy = self.frame_rgb.copy()
                    results = self.model.track(self.frame_rgb , persist=False, verbose=False)#จำกัดจำนวน [0]
                    # size cam
                    h, w, _ = self.frame_rgb.shape

                    if results[0].boxes.id is not None:
                        #id
                        track_ids = results[0].boxes.id.int().cpu()

                        #box
                        boxes = results[0].boxes.xyxy.cpu()

                        #keypoint ตำแหน่งกระดูก
                        result_keypoint = results[0].keypoints.xyn.cpu().numpy()

                        frame_ = results[0].plot()


                        for box , id  in zip(boxes,track_ids):
                            
                            
                            xmin, ymin, xmax, ymax = box

                            xmin = float(xmin)
                            ymin = float(ymin)
                            xmax = float(xmax)
                            ymax = float(ymax)

                            # Calculate the center point of the bounding box
                            px1 = round(xmin + (xmax - xmin) / 2)
                            py1 = round(ymin + (ymax - ymin) / 2)

                            print(f'{px1} : {py1}')

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
                                print(dist)

                                if dist > 0.5:
                                    x_camera = -(px1 - cx) * (dist / fh)  # Reverse the sign for correct direction
                                    y_camera = -(py1 - cy) * (dist / fv)  # Reverse the sign for correct direction
                                    print(f'x: {x_camera} y: {y_camera} z: {dist}')

                                    self.br.sendTransform(
                                        (dist , x_camera, y_camera),  # Translation (x, y, z)
                                        tf.transformations.quaternion_from_euler(0, 0, 0), 
                                        rospy.Time.now(),
                                        "human",  # Child frame
                                        "camera_depth_frame"  # Parent frame
                                    )


                    cv2.imshow("human",frame_)
                    #cv2.imshow("RGB", self.frame_rgb)
                    cv2.imshow("main frame", frame_copy)
                    #เช็คค่า server ที่ต้องทำการส่ง
                    '''
                            
                    if self.direction != 'None':
                        print('Direction : ',self.direction)
                        print('Sucess    : ',self.sucess)
                        #ไม่ส่งค่า Distance
                        print('Distance  : ',self.min_value ," mm")

                        rospy.loginfo('Direction : ',self.direction)
                        rospy.loginfo('Sucess    : ',self.sucess)
                        rospy.loginfo('Distance  : ',self.min_value ," mm")

                        cv2.destroyAllWindows()

                        cv2.imwrite(os.path.join(pack_path,'/temp/human_pose.jpg'),frame_)
                        cv2.imwrite(os.path.join(pack_path,'/temp/hand.jpg'),frame_copy)

                        server_response =  HumanPoseDetectionResponse(self.direction , self.sucess)
                        server_response.success =  self.sucess
                        server_response.direction = self.direction

                        return server_response

                    else:
                        self.direction = "None"
                        self.sucess = False

                    

                        rospy.loginfo(' Not detect hand')


                        print('Direction : ',self.direction)
                        print('Sucess : ',self.sucess)
                        
                    '''
                    cv2.waitKey(1)
                    self.rate.sleep()
                except Exception as e:
                    tb = traceback.format_exc()
                    print(f"Traceback details:\n{tb}")
                    rospy.logerr(e)

    def run(self):
        print("start")
        s = rospy.Service('/rip/turtlebot/vision/human_pose_tf', HumanPoseDetection, self.main_loop)
        rospy.spin()

def main():
    try:
        #ที่อยู่ topic กล้อง
        topic_camera_color = '/camera/rgb/image_color'
        topic_camera_depth = '/camera/depth/image_raw'

        #ที่อยู่ url Ai
        yolov8_pose = pack_path + f'/data/yolo_model/hand/yolov8n-pose.pt'
        HandDetectionNode(topic_camera_color, topic_camera_depth, yolov8_pose , False)
        rospy.spin()
    except rospy.ROSInternalException:
        pass


if __name__ == '__main__':
    main()










