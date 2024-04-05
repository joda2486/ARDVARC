""" 

ARDVARC Flight Software

Author: Tim Behrer
email: Timothy.Behrer@colorado.edu

Description: This is the CV node, it takes in the camera feed from ROS and annotates it for analysis, 
then publishes the annotated feed to the main state machine. It also performs calculations to 
determine the angular position of the ArUco markers. It will then publish the pointing vector 
to an appropriate ROS topic along with a recent sightings boolean to indicate if the marker has
been seen recently.

## Notes
- This node will be executed in parrallel with the main state machine.

## TODO create the ArUco marker detection and annotation function
## TODO create the pointing vector calculation function
## TODO create the recent sightings boolean function
## TODO Clean up the imports and improve the ArUco marker detection and annotation function
## TODO Ensure that coding is sufficiently modular and split into functions
"""




## Imports
import time
import rospy
import genpy
from .config.topic_names import ANNOTATED_CAMERA_FRAMES, RECENT_RGV_SIGHTINGS, UAS_TO_RGV_DIRECTION_VECTORS, CAMERA_FRAMES
from rosardvarc.msg import RecentSighting, UasToRgvDirectionVectorUasFrame
from sensor_msgs.msg import Image
from .functional.process_frame import detect_ArUco_Direction_and_Pose, camera_frame_to_UAS_frame,my_estimatePoseSingleMarkers
import math
from .config import constants
## Imports from the existing ArUco marker detection and annotation function
import sys
import cv2
from cv2 import aruco
import numpy as np
import os
import argparse
from cv_bridge import CvBridge
## TODO Make sure all imports are correct
## TODO Create the callback for the camera frame subscriber.
## TODO Call the write to flash function within this callback


## TODO (02/26 - TB) Finish the cv.py message publishing 

def frame_callback(msg: Image):
    now = rospy.Time.now()
    if (now - msg.header.stamp).to_sec() > 1/2:
        rospy.logdebug(f"CV skipped a frame ({now.to_sec()} vs {msg.header.stamp.to_sec()})")
        return
    
    # rospy.logdebug(f"I AM NOT SKIPPING AHHHHHHHHHHHHHHH")
    frame = bridge.imgmsg_to_cv2(msg)
    
    detection_info = detect_ArUco_Direction_and_Pose(frame)

    ##Annotated Frame Message Definition
    # TODO: Make this something reasonable based on detection_info.annotated_camera_frame (TB 2021-04-07: I think that this is right but im not sure that this is the correct way to do this)
    annotated_frame = bridge.cv2_to_imgmsg(detection_info.annotated_camera_frame, encoding="bgr8")
    pub_frame.publish(annotated_frame)
    rospy.logdebug("CV published an annotated frame")


    if detection_info.rgv_ids is None:
        rospy.logdebug("CV processed a frame but found nothing")
        return

    i = 0
    for rgv_id in detection_info.rgv_ids:
        ##Recent Sighting Message Definition
        sighting = RecentSighting()
        sighting.timestamp = msg.header.stamp
        sighting.rgv_id = rgv_id
        # TODO: Make this something reasonable based on id (TB 2021-04-07: I think that this is right but im not sure that this is the correct way to do this)
        rospy.logdebug("CV published an RGV sighting")
        pub_sightings.publish(sighting)
        # for direction_vector in detection_info.direction_vectors:
        direction_vector_msg = UasToRgvDirectionVectorUasFrame()
        direction_vector_msg.timestamp = msg.header.stamp
        direction_vector_msg.direction = detection_info.direction_vectors[i].flatten().tolist()
        direction_vector_msg.rgv_id = rgv_id
        direction_vector_msg.measurement_source = constants.MEAS_FROM_CAMERA
        # TODO: Make this something reasonable based on direction_vector (TB 2024-02-26: I think that this is right but im not sure that this is the correct way to do this)
        rospy.logdebug("CV published a direction vector")
        pub_vector.publish(direction_vector_msg) 
        i += 1


## Initialize the necessary nodes and the publishers.
rospy.init_node("cv_node")
bridge = CvBridge()
pub_frame = rospy.Publisher(ANNOTATED_CAMERA_FRAMES, Image, queue_size=1)
## TODO Implement the publisher for the recent sightings.
pub_sightings = rospy.Publisher(RECENT_RGV_SIGHTINGS, RecentSighting, queue_size=1)
## TODO Implement the publisher for the pointing vector.
pub_vector = rospy.Publisher(UAS_TO_RGV_DIRECTION_VECTORS, UasToRgvDirectionVectorUasFrame, queue_size=1)
## TODO Implement the subscriber for the camera frame.
sub_frame = rospy.Subscriber(CAMERA_FRAMES, Image, frame_callback)


## Spin until killed
rospy.spin()
