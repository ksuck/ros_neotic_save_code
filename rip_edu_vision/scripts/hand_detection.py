#!/usr/bin/env python3

from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rip_edu_msgs.srv import HandDetection, HandDetectionResponse
import cv2
from rospkg import RosPack
import numpy as np
import os
import pickle
import mediapipe as mp
from ultralytics import YOLO

pack_path = RosPack().get_path('rip_edu_vision')

class HandDetectionNode:
    def __init__(self, cam_color, cam_depth, model_yolo = '' , pickle_model = '', revers_cam = False, ros_rate = 500):
        #ชื่อ node
        rospy.init_node('hand_detection', anonymous=True)
        rospy.Subscriber(cam_color, Image, self.rgb_callback)
        rospy.Subscriber(cam_depth, Image, self.depth_callback)

        #ตั้งค่า mediapipe
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(static_image_mode=False, min_detection_confidence=0.65, min_tracking_confidence=0.55)

        self.bridge = CvBridge()

        #set up pickle dict
        self.labels_dict = {0: 'left', 1: 'right'}

        model_dict = pickle.load(open(pickle_model, 'rb'))
        self.pickle_model = model_dict['model']

        #กลับด้านกล้อง
        self.debug_revers = revers_cam

        #ความเร็วการทำงานกล้อง
        self.rate = rospy.Rate(ros_rate)

        self.model = YOLO(model_yolo)

        self.service = rospy.Service('/rip/turtlebot/vision/hand_detection', HandDetection, self.hand_detection_callback)
        rospy.loginfo("HandDetection Service started!")

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
        self.success = False
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
                    # # Horizontal field of view in degrees
                    horizontal_fov = 58
                    vertical_fov = 45

                    fx = h / (2 * np.tan(np.radians(horizontal_fov / 2)))
                    fy = w / (2 * np.tan(np.radians(vertical_fov / 2)))

                    # Calculate principal point (in pixels)
                    cx = h / 2
                    cy = w / 2

                    if results[0].boxes.id is not None:
                        #id
                        track_ids = results[0].boxes.id.int().cpu()

                        #box
                        boxes = results[0].boxes.xyxy.cpu()

                        #keypoint ตำแหน่งกระดูก
                        result_keypoint = results[0].keypoints.xyn.cpu().numpy()

                        frame_ = results[0].plot()

                    #หาตรงกลางตำแหน่งกระดูกช่วงอก
                        btw_distance = []
                        near_id = 0
                        for res_key , id  in zip(result_keypoint,track_ids):

                            px1 = res_key[5][0] * w
                            py1 = res_key[5][1] * h

                            px2 = res_key[6][0] * w
                            py2 = res_key[6][1] * h

                            if px1 != '' or px2 != '':
                                if px1 != 0 and px2 != 0:
                                    ctx_p = (px1 + px2) /2
                                    cty_p = (py1 + py2) /2

                                    cv2.putText(frame_copy, f"{id}", (abs(int(ctx_p)), abs(int(cty_p))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                                    cv2.circle(frame_copy , (abs(int(ctx_p)), abs(int(cty_p))), 5, (255, 0, 255), cv2.FILLED)

                                    if (int(ctx_p) >= 0 and int(ctx_p) <= (w-1)) or int(cty_p) >= 0 and int(cty_p) <= (h-1):
                                        dist =  self.frame_depth[int(cty_p-1) , int(ctx_p)] #mm
                                        if dist < 2000:
                                            btw_distance.append((int(id),dist))

                            #หาคนที่ใกล้ที่สุดมาใช้
                            if len(btw_distance) > 0:
                                #หาค่าที่น้อยที่สุด
                                min_value = min(pair[1] for pair in btw_distance)
                                self.min_value = min_value

                                #เอา id มาใช้เพื่อกำหนด box มา crop
                                for index, item in enumerate(btw_distance):
                                    if item[1] == min_value:
                                        near_id = item[0]

                    cropped_image = np.ones((320, 320, 3), dtype=np.uint8) * 255
                    #เอาขนาดคนมาใช้คนที่ใกล้
                    if near_id > 0:
                        x1, y1, x2, y2 = boxes[near_id - 1]
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                        cropped_image = frame_copy[y1:y2, x1:x2]


                        crop_x, crop_y, _ = cropped_image.shape
                        data_aux = []
                        x_ = []
                        y_ = []

                        #hand detection
                        hand_detec = self.hands.process(cropped_image)
                        if  hand_detec.multi_hand_landmarks:
                            for hand_landmarks in hand_detec.multi_hand_landmarks:
                                self.mp_drawing.draw_landmarks(
                                    cropped_image,
                                    hand_landmarks,
                                    self.mp_hands.HAND_CONNECTIONS,
                                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                                    self.mp_drawing_styles.get_default_hand_connections_style()
                                )


                                for i in range(len(hand_landmarks.landmark)):
                                    x = hand_landmarks.landmark[i].x
                                    y = hand_landmarks.landmark[i].y

                                    x_.append(x)
                                    y_.append(y)


                                for i in range(len(hand_landmarks.landmark)):
                                    x = hand_landmarks.landmark[i].x
                                    y = hand_landmarks.landmark[i].y
                                    data_aux.append(x - min(x_))
                                    data_aux.append(y - min(y_))


                            hand_x1 = int(min(x_) * crop_x) - 10
                            hand_y1 = int(min(y_) * crop_y) - 10

                            hand_x2 = int(max(x_) * crop_x) - 10
                            hand_y2 = int(max(y_) * crop_y) - 10

                            try:
                                prediction = self.pickle_model.predict([np.asarray(data_aux)])

                                if int(prediction[0]) < 2:
                                    predicted_character = self.labels_dict[int(prediction[0])]

                                    if int(prediction[0]) == 0:
                                        self.direction = predicted_character
                                        self.success = True

                                    if int(prediction[0]) == 1:
                                        self.direction = predicted_character
                                        self.success = True

                                    cv2.putText(cropped_image, predicted_character, (hand_x1, hand_y1), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)

                            except:
                                pass

                    cv2.imshow("human",frame_)
                    cv2.imshow("RGB", self.frame_rgb)
                    cv2.imshow("main frame", frame_copy)
                    #เช็คค่า server ที่ต้องทำการส่ง
                    if self.direction != 'None':
                        print('Direction : ',self.direction)
                        print('success    : ',self.success)
                        #ไม่ส่งค่า Distance
                        print('Distance  : ',self.min_value ," mm")

                        # rospy.loginfo('Direction : ',self.direction)
                        # rospy.loginfo('success    : ',self.success)
                        # rospy.loginfo('Distance  : ',self.min_value ," mm")

                        cv2.destroyAllWindows()

                        cv2.imwrite(os.path.join('/home/naja/Home/src/rip_edu_vision/human_pose.jpg'),frame_)
                        cv2.imwrite(os.path.join('/home/naja/Home/src/rip_edu_vision/hand.jpg'),frame_copy)

                        # server_response =  HandDetectionResponse(self.direction , self.success)
                        # server_response.success =  self.success
                        # server_response.direction = self.direction
                        # server_response =  HandDetectionResponse("right" , True)

                        # return HandDetectionResponse("right" , True)
                        return HandDetectionResponse(str(self.direction) , self.success)

                    else:
                        self.direction = "None"
                        self.success = False

                        rospy.loginfo(' Not detect hand')


                        print('Direction : ',self.direction)
                        print('success : ',self.success)

                    cv2.waitKey(1)
                    self.rate.sleep()
                except Exception as e:
                    rospy.logerr(e)

    # def run(self):
    #     print("start")
    #     s = rospy.Service('/rip/turtlebot/vision/hand_detection', HandDetection, self.main_loop)
    #     rospy.spin()

def main():
    try:
        #ที่อยู่ topic กล้อง
        topic_camera_color = '/camera/rgb/image_color'
        topic_camera_depth = '/camera/depth/image_raw'

        #ที่อยู่ url Ai
        yolov8_pose = pack_path + f'/data/yolo_model/hand/yolov8n-pose.pt'

        #model Classifier
        pickle_model = pack_path + f'/data/yolo_model/hand/model_lucky.p'

        HandDetectionNode(topic_camera_color, topic_camera_depth, yolov8_pose , pickle_model , False)
        rospy.spin()
    except rospy.ROSInternalException:
        pass


if __name__ == '__main__':
    main()










