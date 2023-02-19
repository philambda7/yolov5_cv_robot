import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.time import Time

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 

from cv_bridge import CvBridge

from message_filters import Subscriber, TimeSynchronizer

import numpy as np
import torch
import cv2

import sys
YOLO_ROOT = '/app/ws/yolov5'
if YOLO_ROOT not in sys.path:
    sys.path.append(YOLO_ROOT)  # add ROOT to PATH

from models.common import DetectMultiBackend
from utils.general import non_max_suppression
from utils.augmentations import letterbox

class Inferator(Node):

    def __init__(self):
        super().__init__('yolov5_inferator')
    
        # subscribe to image and point cloud data synchronously
        self.image_sub = Subscriber(self, Image, 'camera/image_raw')
        self.pcd_sub = Subscriber(self, PointCloud2, 'camera/points')
        self.ts = TimeSynchronizer([self.image_sub, self.pcd_sub], 1)
        self.ts.registerCallback(self.callback)

        self.get_logger().info('Subscribed!')

        # load yolov5 model
        MODEL_PATH = '/app/ws/yolov5/custom_model.pt'
        self.device = 'cpu'
        self.model = DetectMultiBackend(MODEL_PATH, device=self.device)
        self.stride = self.model.stride
        self.names = self.model.names

        # some yolov5 params
        self.conf_thresh = 0.25  # conf_thresh is set very low, detecting real obstacles is more important than not detecting fake ones
        self.iou_thresh = 0.45
        self.classes = None
        self.agnostic_nms=False
        self.max_det = 1000
        
        # circle to inflate detections
        r = 0.07
        d = 100
        self.CIRCLE = np.array([[r*np.cos(2*np.pi*i/d),r*np.sin(2*np.pi*i/d),0] for i in range(d)]) 
        
        # computer-vision bridge
        self.br = CvBridge()

        self.get_logger().info('Computer-vision ready!')

        #  create publisher
        self.publisher_ = self.create_publisher(PointCloud2, 'yolov5/scan', 10)

        self.get_logger().info('Inferator initialized!')

        

    def callback(self, image, pcd):
        self.get_logger().info("Got image!")

        # convert image to cv2
        frame = self.br.imgmsg_to_cv2(image)
        h,w,ch = frame.shape
        
        # convert pointcloud to numpy
        pcd_np = point_cloud2.read_points_numpy(pcd)

        # bring frame to the right format for yol0v5 inference
        im = self.prepare_frame(frame)
        
        # inference
        pred_raw = self.model(im, augment = 0)

        # non max surpression + thresholding
        pred = non_max_suppression(pred_raw, self.conf_thresh, self.iou_thresh, self.classes, self.agnostic_nms, max_det=self.max_det)[0].numpy()

        # find xyz coodrs of obstacles in depth camera coordinate system

        centers = ((pred[:,2:4]+pred[:,:2])/2).astype(int)
        obstacles = np.array([pcd_np[c[0]+w*c[1],:3] for c in centers])
        self.get_logger().info('Found obstacles at ' +  np.array2string(obstacles))

        if len(obstacles)>0:
            
            # hacky solution: obstacles are inflated so nav2 plugin detects them better. 
            obstacles = np.concatenate([p + self.CIRCLE for p in obstacles], axis =0)
            
            fields = [
                PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1)]
            points = np.array([np.append(a,2.3509886e-38) for a in obstacles]) 

            header = Header(frame_id='camera_link_optical')
            
            # detection copies the time stamp of the image to count out lag from processing
            header.stamp = image.header.stamp 

            self.publisher_.publish(point_cloud2.create_cloud(header, fields, points))


    def prepare_frame(self,frame):

        im = letterbox(frame, 640, stride=self.stride, auto=1)[0]
        im = im.transpose((2, 0, 1))[::-1]
        im = np.ascontiguousarray(im)
        im = torch.from_numpy(im).to(self.device)
        im = im.float()  #uint8 to fp32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        im = im[None]  # expand for batch dim
        return im 

def main(args=None):
    rclpy.init(args=args)
    inferator = Inferator()
    rclpy.spin(inferator)

    inferator.destroy_node()
    inferator.shutdown()

if __name__ == '__main__':
    main()