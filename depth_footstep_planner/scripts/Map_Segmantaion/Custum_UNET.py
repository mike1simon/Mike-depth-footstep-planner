#!/usr/bin/env python3

import os
import numpy as np
from numpy.core.defchararray import mod
import rospy

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.functional import Tensor

import torchvision.transforms as transforms
import rospkg
from nav_msgs.msg import OccupancyGrid, MapMetaData
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

# from typing import List # for specifing List type and autocomplete for it
class conv_block(nn.Module):
    def __init__(self, in_c, out_c,kernal1=3,kernal2=3):
        super().__init__()

        self.conv1 = nn.Conv2d(in_c, out_c, kernel_size=kernal1, padding=int(kernal1/2))
        self.bn1 = nn.BatchNorm2d(out_c)

        self.conv2 = nn.Conv2d(out_c, out_c, kernel_size=kernal2, padding=int(kernal2/2))
        self.bn2 = nn.BatchNorm2d(out_c)

        self.relu = nn.ReLU()

    def forward(self, inputs):
        x = self.conv1(inputs)
        x = self.bn1(x)
        x = self.relu(x)

        x = self.conv2(x)
        x = self.bn2(x)
        x = self.relu(x)

        return x

class encoder_block(nn.Module):
    def __init__(self, in_c, out_c,kernal1=3,kernal2=3):
        super().__init__()

        self.conv = conv_block(in_c, out_c,kernal1=3,kernal2=3)
        self.pool = nn.MaxPool2d((2, 2))

    def forward(self, inputs):
        x = self.conv(inputs)
        p = self.pool(x)

        return x, p
    
class decoder_block(nn.Module):
    def __init__(self, in_c, out_c):
        super().__init__()

        self.up = nn.ConvTranspose2d(in_c, out_c, kernel_size=2, stride=2, padding=0)
        self.conv = conv_block(out_c+out_c, out_c)

    def forward(self, inputs, skip):
        x = self.up(inputs)
        x = torch.cat([x, skip], axis=1)
        x = self.conv(x)

        return x

class Custom_UNET_Model(nn.Module):
    def __init__(self):
        super().__init__()
        
        """ Encoder """
        self.e1 = encoder_block(1, 64,kernal1=7,kernal2=5)
        self.e2 = encoder_block(64, 128, kernal1=5, kernal2=3)

        """ Bottleneck """
        self.b1 = conv_block(128, 256)

        """ Decoder """
        self.d1 = decoder_block(256, 128)
        self.d2 = decoder_block(128, 64)

        """ Classifier """
        self.outputs = nn.Conv2d(64, 1, kernel_size=1, padding=0)
        self.last_layer_activation = nn.Sigmoid()

    def forward(self, inputs):
        
        """ Encoder """
        s1, p1 = self.e1(inputs)
        s2, p2 = self.e2(p1)

        """ Bottleneck """
        b1 = self.b1(p2)

        """ Decoder """
        d1 = self.d1(b1, s2)
        d2 = self.d2(d1, s1)

        """ Classifier """
        outputs = self.outputs(d2)
        outputs = self.last_layer_activation(outputs)

        return outputs

# data_transforms = transforms.Compose([
#         transforms.Normalize((65534.0/2.0), (65534.0/2.0))
#         ])
class Map_Seg_Server:
    def __init__(self,model_name = "model2.pth", root_path = None,
             device = None, data_transforms = None, 
             output_pub_name = "/model_output", vis_pub_name = "/model_output_vis",
             sub_name = "/map16bit", service_name = "/Map_Seg_service"):
        self.model_name = model_name
        rospack = rospkg.RosPack()
        if root_path is None:
            self.root_path = os.path.join(rospack.get_path('depth_footstep_planner') , "model")
        else:
            self.root_path = root_path
        self.model_output_path = os.path.join(self.root_path,"model_output")
        self.model_original_path = os.path.join(self.root_path,"original")
        self.model_input_path = os.path.join(self.root_path,"input")
        self.original_output_path = os.path.join(self.root_path,"output")
        self.model_path = os.path.join(self.root_path,"models",self.model_name)
        
        if device is None:
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        else:
            self.device = device
        if data_transforms is None:
            self.data_transforms = transforms.Compose([
                transforms.Normalize((65534.0/2.0), (65534.0/2.0))      ])
        else:
            self.data_transforms = data_transforms
        self.criterion = nn.BCELoss()

        self.load_model()

        self.output_pub = rospy.Publisher(output_pub_name,Image,queue_size=10,latch=True)
        self.vis_pub = rospy.Publisher(vis_pub_name,OccupancyGrid,queue_size=10,latch=True)
        self.Map_Seg_sub = rospy.Subscriber(sub_name, Image, self.Map_Seg_cb)
        # self.Service = rospy.Service(service_name, My_Service_Class)
    
    def numpy_to_occupancy_grid(self, arr: np.ndarray, info=None):
        if not len(arr.shape) == 2:
                raise TypeError('Array must be 2D')
        if not arr.dtype == np.int8:
                raise TypeError('Array must be of int8s')

        grid = OccupancyGrid()
        if isinstance(arr, np.ma.MaskedArray):
                # We assume that the masked value are already -1, for speed
                arr = arr.data
        grid.data = arr.ravel()
        grid.info = info or MapMetaData()
        grid.info.height = arr.shape[0]
        grid.info.width = arr.shape[1]
        grid.info.resolution = 0.01
        grid.header = Header()
        grid.header.frame_id = "map"
        grid.header.seq = 1
        grid.header.stamp = rospy.Time.now()
        return grid

    def output_to_msg(self, output: np.ndarray):
        output: np.ndarray = 255* output.astype("uint8")
        bridge = CvBridge()
        output_msg = bridge.cv2_to_imgmsg(output, encoding="8UC1")
        return output_msg

    def scale_image(self, image : np.ndarray):
        # grab the image dimensions
        h = image.shape[0]
        w = image.shape[1]
        new_image: np.ndarray = np.zeros((int(image.shape[0]/2),int(image.shape[1]/2)),image.dtype)
        # loop over the image, pixel by pixel
        for y in range(0, h,2):
            for x in range(0, w,2):
                # threshold the pixel
                # new_image[int(y/2), int(x/2)] = image[y, x,0]
                new_image[int(y/2), int(x/2)] = image[y, x]
                
        # return the thresholded image
        return new_image

    def load_model(self,model_name = None, root_path = None, device = None, criterion = None):
        if root_path is not None:
            self.root_path = root_path
            self.model_output_path = os.path.join(self.root_path,"model_output")
            self.model_original_path = os.path.join(self.root_path,"original")
            self.model_input_path = os.path.join(self.root_path,"input")
            self.original_output_path = os.path.join(self.root_path,"output")
        self.model_path = os.path.join(self.root_path,"models",self.model_name)
        if device is not None:
            self.device = device
        self.model = Custom_UNET_Model().to(self.device)
        if criterion is not None:
            self.criterion = criterion
        rospy.loginfo("Feasible Footsteps Segmentation Node Loaded UNET Model from: %s", self.model_path)
        self.model.load_state_dict(torch.load(self.model_path))
        # self.model.load_state_dict(self.model_path)
        return self.model


    def run_model(self, msg: Image, model = None):
        bridge = CvBridge()
        original_image = bridge.imgmsg_to_cv2(msg, msg.encoding)
        # print("recieved input size: ",original_image.shape)
        input_image = self.scale_image(original_image)
        input = torch.from_numpy(input_image.astype(np.float32))
        input = input.view(1,input.shape[0],input.shape[1])
        input = self.data_transforms(input)
        input = input.view(1,input.shape[0],input.shape[1],input.shape[2])
        input = input.to(self.device)
        # output = output.to(device)

        if model is not None:
            self.model = model
        self.model.eval()
        # Forward pass
        with torch.no_grad():
            model_output: Tensor = self.model(input)
        output_thresholded: np.ndarray = np.round(model_output.numpy()[0,0],0)
        # print("thresholded output shape and type", output_thresholded.shape, output_thresholded.dtype)

        return output_thresholded

    def Map_Seg_cb(self, msg: Image):
        rospy.loginfo("Feasible Footsteps Segmentation Node Recieved Map (Request).")
        self.output: np.ndarray =  self.run_model(msg)
        self.output_vis: np.ndarray =  (50* self.output).astype("int8")
        # print("output shape: ", self.output.shape, "data type: ", self.output.dtype)  
        # print("output_vis[15:25,15:25]: ")  
        # print(self.output_vis[15:25,15:25])  
        self.output_vis = self.output_vis.T
        self.vis_msg = self.numpy_to_occupancy_grid(self.output_vis)
        self.vis_pub.publish(self.vis_msg)
        self.output_msg = self.output_to_msg(self.output)
        self.output_pub.publish(self.output_msg)
        rospy.loginfo("Feasible Footsteps Segmentation Node Publishing Results(Model Outputs).")
