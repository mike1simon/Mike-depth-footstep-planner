#!/usr/bin/env python3

import sys
import rospy
from geometry_msgs.msg import PoseArray,Pose,PoseStamped
from std_msgs.msg import Float64MultiArray
from depthmap_humanoid_msgs.msg import GreyScaleMap16bit
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np

from cv_bridge import CvBridge
from Map_Segmantaion.Custum_UNET import Map_Seg_Server

# global output_pub, criterion, device, model
    
if __name__ == "__main__":
    
    try:

        rospy.init_node('Planner', anonymous=True)
        map_segmenter = Map_Seg_Server()
        
        # input_sub = rospy.Subscriber("map", GreyScaleMap16bit, callback=run_model)
        # output_pub = rospy.Publisher("model_output", Float64MultiArray,queue_size=1,latch=True)
        
        # Device configuration
        # device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        # model = loadModel("model.pth").to(device)
        # criterion = nn.BCELoss()

        # x = torch.rand(5, 3)
        # print(x)
        # print(torch.cuda.is_available())

        print("Everything OK!")
        rospy.spin()
        print("closing now")
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass