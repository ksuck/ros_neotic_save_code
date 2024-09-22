#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
from rospkg import RosPack


pack_path = RosPack().get_path('rip_edu_vision')
class estimate() :
    """
    estimating size from bounding box.
    ---
    ประมาณขนาดจากขอบ bounding box
    
    """
    def __init__(self, ix : int, iy : int, fx : float, fy : float) -> None :
        rospy.init_node('human_height_node', anonymous=True)
        rospy.Subscriber('/camera/rgb/image_color', Image, self.rgb_callback)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)

        self.bridge = CvBridge()
        self.rate = rospy.Rate(500)

        """init value.
        
        ตั้งค่าการใช้งาน
        
        Args:
            ix (int): wide of default camera image. ความกว้างรูปมาตรฐานของกล้อง
            iy (int): high of default camera image. ความสูงรูปมาตรฐานของกล้อง
            fx (float): Horizontal FOV of camera. FOV แนวแกนนอนของกล้อง
            fy (float): Vertical FOV of camera. FOV แนวแกนตั้งของกล้อง
        """
        
        self.ix = ix
        self.iy = iy
        self.fx = fx
        self.fy = fy
        self.pixel_est_x = fx / ix  # deg/pix
        self.pixel_est_y = fy / iy  # deg/pix

        yolov8_pose = pack_path + f'/data/yolo_model/hand/yolov8n-pose.pt'
        self.model = YOLO(yolov8_pose)

        h, w, _ = self.frame_rgb.shape

        while not rospy.is_shutdown():
            if hasattr(self, 'frame_rgb') and hasattr(self, 'frame_depth'):

                results = self.model.track(self.frame_rgb , persist=False, verbose=False)#จำกัดจำนวน [0]

                if results[0].boxes.id is not None:
                    track_ids = results[0].boxes.id.int().cpu()
                    boxes = results[0].boxes.xyxy.cpu()

                    for box , id  in zip(boxes,track_ids):
                            
                        xmin, ymin, xmax, ymax = box

                        xmin = float(xmin)
                        ymin = float(ymin)
                        xmax = float(xmax)
                        ymax = float(ymax)

                        px1 = round(xmin + (xmax - xmin) / 2)
                        py1 = round(ymin + (ymax - ymin) / 2)

                        dist = self.frame_depth[abs(int(py1)), abs(int(px1))] 

                        print(self.estimate_hight(ymax, ymin, dist)-60)

            self.rate.sleep()

    def rgb_callback(self, data):
        try:
            frame_rgb = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.frame_rgb = frame_rgb
          
        except :
            pass

    def depth_callback(self, data):
        try:
            frame_depth = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            self.frame_depth = frame_depth
          
        except: 
            pass

    def estimate_hight(self, iy1 : int, iy2 : int, distant : float) -> float :
        """estimate hight
        
        ประมาณความสูง

        Args:
            iy1 (int): highest point of the bounding box. จุดสูงสุงของ bounding box
            iy2 (int): lowest point of the bounding box. จุดต่ำสุงของ bounding box
            distant (float): distant from camera to estimated object. ระยะทางระหว่างกล้องไปยังเป้าหมายที่จะประมาณขนาด

        Returns:
            float: estimate hight of the object. ความสูงของวัตถุโดยประมาณ
        """
        div_iy1 = int((self.iy / 2) - iy1)
        div_iy2 = int(iy2 - (self.iy / 2))
        est_y1 = distant * np.tan(np.radians(div_iy1 * self.pixel_est_y))
        est_y2 = distant * np.tan(np.radians(div_iy2 * self.pixel_est_y))
        est_hight = est_y1 + est_y2
        return est_hight
    
    def estimate_wide(self, ix1 : int, ix2 : int, distant : float) -> float :
        """estimate wide
        
        ประมาณความกว้าง

        Args:
            ix1 (int): far left point of the bounding box. จุดซ้านสุดของ bounding box
            ix2 (int): far right point of the bounding box. จุดขวาสุงของ bounding box
            distant (float): distant from camera to estimated object. ระยะทางระหว่างกล้องไปยังเป้าหมายที่จะประมาณขนาด

        Returns:
            float: estimate wide of the object. ความกว้างของวัตถุโดยประมาณ
        """
        div_ix1 = int((self.iy / 2) - ix1)
        div_ix2 = int(ix2 - (self.iy / 2))
        est_x1 = distant * np.tan(np.radians(div_ix1 * self.pixel_est_x))
        est_x2 = distant * np.tan(np.radians(div_ix2 * self.pixel_est_x))
        est_wide = est_x1 + est_x2
        return est_wide
            
if __name__ == "__main__" :
    est = estimate(640,480, 57, 43)
    
    