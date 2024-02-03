#!/usr/bin/env python3

import rospy
from Map_Segmantaion.Custum_UNET import Map_Seg_Server

if __name__ == "__main__":

    try:

        rospy.init_node('depthmap_feasible_footstep_segmentation_node',
                        anonymous=True)
        map_segmenter = Map_Seg_Server()
        rospy.loginfo("Depthmap Feasible Footstep Segmentation Node Started!")
        rospy.spin()
        rospy.loginfo("Depthmap Feasible Footstep Segmentation\
            Node Closing Now")
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
