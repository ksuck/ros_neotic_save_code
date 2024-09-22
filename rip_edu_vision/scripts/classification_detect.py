#!/usr/bin/env python3

from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rip_edu_msgs.srv import Classification, ClassificationResponse
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


class classification_node:
    def __init__(self,  model_yolo_pose = '' , model_yolo = '', model_2 = '' , model_3 = ''  , model_4 = '' , count_image = 4 ,revers_cam = False, ros_rate = 500):
        #ชื่อ node
        rospy.init_node('classification_node', anonymous=True)
       
        
        self.bridge = CvBridge()

        #กลับด้านกล้อง
        self.debug_revers = revers_cam

        #ความเร็วการทำงานกล้อง
        self.rate = rospy.Rate(ros_rate)

        self.human_pose = YOLO(model_yolo_pose)
        self.object = YOLO(model_yolo)
        self.mask = YOLO(model_2)
        self.glasses = YOLO(model_3)
        self.cap = YOLO(model_4)

        self.count_img = count_image

        #TF Broadcaster
        self.br = tf.TransformBroadcaster()
        self.service = rospy.Service('/rip/turtlebot/vision/classification_detect', Classification, self.classification_callback)
        rospy.loginfo("classification_node Service started!")

    def classification_callback(self , req):
        self.sucess = False

        #ตำแหน่งของคนในภาพ
        human_pos = []
        #ตำแหน่งหน้ากาก
        mask_check = []
        #ตำแหน่งแว่น
        glasses_check = []
        #ตำแหน่งหมวก
        cap_check = []
        #object ที่เจอ
        obj_his = [] 



        data_respone = []


        have_img = 0

        while not rospy.is_shutdown():
                try:
                    
                    for i in range(self.count_img):
                        # Construct the filename dynamically
                        image_filename = f"human_pose_detection{i}.jpg"
                        image_path = os.path.join(pack_path, image_filename)



                        # Check if the file exists
                        if os.path.exists(image_path):
                            # Read the image using OpenCV
                            image = cv2.imread(image_path)

                            if image is not None:
                                h , w , _ = image.shape
                                image_copy = image.copy()

                                #human pos
                                res_human_pos = self.human_pose.track(image, persist=False, verbose=False)
                                #--------------------------------------------------------------------#
                                if res_human_pos[0].boxes.id is not None:
                                    #id
                                    track_ids = res_human_pos[0].boxes.id.int().cpu()

                                    #box
                                    boxes = res_human_pos[0].boxes.xyxy.cpu()

                                   

                                    for box , id  in zip(boxes,track_ids):
                                        
                                        xmin, ymin, xmax, ymax = box

                                        xmin = float(xmin)
                                        ymin = float(ymin)
                                        xmax = float(xmax)
                                        ymax = float(ymax)

                                        # Calculate the center point of the bounding box
                                        px1 = round(xmin + (xmax - xmin) / 2)
                                        py1 = round(ymin + (ymax - ymin) / 2)

                                        cv2.circle(image_copy, (abs(int(px1)), abs(int(py1))), 5, (255, 0, 255), cv2.FILLED)

                                        human_pos.append((image_filename,int(id),px1,py1))

                                #mask
                                res_Mask = self.mask.track(image, persist=False, verbose=False)
                                #--------------------------------------------------------------------#
                                if res_Mask[0].boxes.id is not None:
                                    #class
                                    class_indices_mask = res_Mask[0].boxes.cls.cpu().numpy()  # ดึงดัชนีคลาสที่ตรวจจับได้

                                    res_frame_mask = res_Mask[0].plot()
                                    
                                    #id
                                    track_mask_ids = res_Mask[0].boxes.id.int().cpu()

                                    #box
                                    #boxes = res_Mask[0].boxes.xyxy.cpu()

                                    for id  , class_mask in zip(track_mask_ids, class_indices_mask):

                                        cv2.imwrite(os.path.join(pack_path,f'mask{int(id)}.jpg'),res_frame_mask)

                                        if int(class_mask) == 0:
                                            mask_check.append((image_filename,int(id),True))

                                        elif int(class_mask) == 1:
                                            mask_check.append((image_filename,int(id),False))
                                else:
                                    mask_check.append((image_filename,int(id),False))
                                #--------------------------------------------------------------------#
                                #glasses
                                res_glasses = self.glasses.track(image, persist=False, verbose=False)
                                if res_glasses[0].boxes.id is not None:
                                    #class
                                    class_indices_glass = res_glasses[0].boxes.cls.cpu().numpy()  # ดึงดัชนีคลาสที่ตรวจจับได้
                                    #id
                                    track_glass_ids = res_glasses[0].boxes.id.int().cpu()

                                    res_frame_glasses = res_glasses[0].plot()

                                    #box
                                    for id , class_glass in zip(track_glass_ids, class_indices_glass):
                                        cv2.imwrite(os.path.join(pack_path,f'glasses{int(id)}.jpg'),res_frame_glasses)

                                        if int(class_glass) == 0:
                                            glasses_check.append((image_filename,int(id),True))

                                else:
                                    glasses_check.append((image_filename,int(id),False))
                                
                                #--------------------------------------------------------------------#
                                #cap
                                res_cap = self.cap.track(image, persist=False, verbose=False)
                                if res_cap[0].boxes.id is not None:
                                    #class
                                    class_indices_cap = res_cap[0].boxes.cls.cpu().numpy()  # ดึงดัชนีคลาสที่ตรวจจับได้

                                    #id
                                    track_cap_ids = res_cap[0].boxes.id.int().cpu()

                                    res_frame_cap = res_cap[0].plot()

                                    #box
                                    for id , class_cap in zip(track_cap_ids , class_indices_cap):
                                        cv2.imwrite(os.path.join(pack_path,f'cap{int(id)}.jpg'),res_frame_cap)
                                        if int(class_cap) == 0:
                                            cap_check.append((image_filename,int(id),True))

                                        elif int(class_cap) == 1:
                                            cap_check.append((image_filename,int(id),False))
                                else:
                                    cap_check.append((image_filename,int(id),False))
                                #-------------------------------------------------------------------#
                                res_object = self.object.track(image, persist=False, verbose=False)
                                
                                if res_object[0].boxes.id is not None:

                                    #id
                                    track_ids_obj = res_object[0].boxes.id.int().cpu()
                                    #name
                                    name_obj = res_object[0].names
                                    #box
                                    boxes_obj = res_object [0].boxes.xyxy.cpu()
                                    #class
                                    class_indices_obj = res_object[0].boxes.cls.cpu().numpy()
                                

                                    #print(class_indices_obj)

                                    res_frame_obj= res_object[0].plot()
                                    for box , id , obj_class in zip(boxes_obj,track_ids_obj,class_indices_obj):
                                        if obj_class >= 1:
                                            cv2.imwrite(os.path.join(pack_path,f'obj_new{id}.jpg'),res_frame_obj)
                                            xmin, ymin, xmax, ymax = box        
                                            xmin = float(xmin)
                                            ymin = float(ymin)
                                            xmax = float(xmax)
                                            ymax = float(ymax)
                                            # Calculate the center point of the bounding box
                                            px2 = round(xmin + (xmax - xmin) / 2)
                                            py2 = round(ymin + (ymax - ymin) / 2)
                                            nearby_copy = image.copy()
                                            cv2.circle(nearby_copy, (abs(int(px2)), abs(int(py2))), 5, (255, 0, 255), cv2.FILLED)
                                            '''
                                            result_x = centerx - px2
                                            result_y = centery - py2
                                            result_x = pow(result_x,2)
                                            result_y  = pow(result_y ,2)
                                            distance = math.sqrt(result_x + result_y)
                                            '''

                            
                                            obj_class_name = name_obj[int(obj_class)]

                                            
                                            obj_his.append((image_filename , obj_class_name, px2 , py2))
                                
                            else:
                                print(f"Failed to read {image_filename}")
                        else:
                            print(f"{image_filename} not found, skipping...")



                     #--------------------------------------------------------------------
                    
                    # หา distance ที่ใกล้ แยกคนใน รูปที่ 1 เลขคน 1 
                    counting = []
                    #หาจำนวนรูปว่ามีกี่รูปที่เจอวัตถุ
                    for i in range(self.count_img):
                        image_filename = f"human_pose_detection{i}.jpg"
                        image_path = os.path.join(pack_path, image_filename)
                        if os.path.exists(image_path):
                            # Read the image using OpenCV
                            image = cv2.imread(image_path)

                            if image is not None:
                                # Construct the filename dynamically

                                count = 0
                                for data in enumerate(human_pos):
                                    if data[1][0] == f"human_pose_detection{i}.jpg":
                                        count = count + 1

                                counting.append(count) #จำนวนที่เป็น 0 - 4 

                                #print(counting)

                            else:
                                print(f"Failed to read {image_filename}")
                        else:
                            print(f"{image_filename} not found, skipping...")

                    #หาวัตถุที่ใกล้ที่สุด
                    for img_count in range(self.count_img):
                        image_filename = f"human_pose_detection{img_count}.jpg"
                        image_path = os.path.join(pack_path, image_filename)

                        #เช็คว่ามีรูป
                        have_img = have_img + 1
                        

                        if os.path.exists(image_path):
                            # Read the image using OpenCV
                            image = cv2.imread(image_path)

                            if image is not None:
                                print('-----------------------')
                                print(f'รูปที่ {image_filename}')
                    
                                for dip in range(counting[img_count]):
                                    print(f'คน {int(dip)+1}')
                                    print(f'ตำแหน่งของคน     {human_pos[img_count]}')
                                    print(f'ตำแหน่ง X    {human_pos[img_count][2]}')
                                    print(f'ตำแหน่ง Y    {human_pos[img_count][3]}')
                                    print('-----------------------')

                                    #ตัวแปรคน
                                    human_start_x = human_pos[img_count][2]
                                    human_start_y = human_pos[img_count][3]

                                    #####################################################
                                    
                                    mask_data = False
                                    for mask_zip in range(len(mask_check)):  
                                        if str(mask_check[mask_zip][0]) == image_filename and int(mask_check[mask_zip][1]) == int(dip+1):
                                            print(f'คนใส่แมส {mask_check[mask_zip][2]}')
                                            mask_data = mask_check[mask_zip][2]

                                    gls_data = False
                                    for gls_zip in range(len(glasses_check)):  
                                        if str(glasses_check[gls_zip][0]) == image_filename and int(glasses_check[gls_zip][1]) == int(dip+1):
                                            print(f'คนใส่แว่น {glasses_check[gls_zip][2]}')
                                            gls_data = glasses_check[gls_zip][2]

                                    cap_data = False
                                    for cap_zip in range(len(cap_check)):  
                                        if str(cap_check[cap_zip][0]) == image_filename and int(cap_check[cap_zip][1]) == int(dip+1):
                                            print(f'คนใส่หมวก {cap_check[cap_zip][2]}')  
                                            cap_data = cap_check[cap_zip][2]                      

                                    #####################################################

                                    #ตัวแปรเก็บ
                                    result_near = 999999999

                                    nearby_result = ['None']

                                    for object_count in range(len(obj_his)):            
                                        if str(obj_his[object_count][0]) == image_filename:
                                            if obj_his[object_count] is not None:
                                                print(f'ที่อยู่ในรูป {obj_his[object_count][1]}')
                                                print(f'ตำแหน่ง X    {obj_his[object_count][2]}')
                                                print(f'ตำแหน่ง Y    {obj_his[object_count][3]}')

                                                #ตัวแปร วัตถุ

                                                obj_ctx = obj_his[object_count][2]
                                                obj_cty = obj_his[object_count][3]

                                                result_x = human_start_x - obj_ctx 
                                                result_y = human_start_y - obj_cty

                                                result_x = pow(result_x ,2)
                                                result_y = pow(result_y ,2)

                                                distance = math.sqrt(result_x + result_y)


                                                if distance < result_near:
                                                    result_near = distance
                                                    nearby_result[0] = obj_his[object_count][1]
                                            else:
                                                nearby_result[0] = 'None'
                                                result_near = 9999999999

                                        
                                    print(f'ค่าที่ใกล้ที่สุด {int(result_near)} วัตถุชื่อ {nearby_result[0]}')
                                    print('--------------------')

                                    data_respone.append((mask_data,gls_data,cap_data,nearby_result[0]))
                            else:

                                print('ไม่มีรูป')

                                

                        



                    #หาจำนวนคนในรูป
                    #print(counting)
                    
                    '''
                    test array 
                    print("คน")
                    print(human_pos)
                    print("แมส")
                    print(mask_check)
                    print("แว่น")
                    print(glasses_check)
                    print("หมวก")
                    print(cap_check)
                    print("ของใกล้ๆ")
                    print (obj_his)
                    '''

                    
                    if have_img > 0:
                        print(data_respone)


                        data_respone = list(data_respone)
                        arr_res = np.array(data_respone)

                        respone = ClassificationResponse()
                        respone.list_of_class = arr_res.flatten()
                        respone.success = True
                                    
                        return respone
                    
                    else:
                        respone = ClassificationResponse()
                        respone.list_of_class = []
                        respone.success = False

                        return respone
 
                    
                
                    
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
        yolov8n = pack_path + f'/data/yolo_model/hand/yolov8n.pt'
        Mask = pack_path + f'/data/yolo_model/hand/Mask.pt'
        glasses = pack_path + f'/data/yolo_model/hand/glasses.pt'
        cap = pack_path + f'/data/yolo_model/hand/Cap.pt'

        #ที่อยู่รูปภาพ


        classification_node(yolov8_pose, yolov8n, Mask , glasses , cap ,4, False)
        rospy.spin()
    except rospy.ROSInternalException:
        pass


if __name__ == '__main__':
    main()










