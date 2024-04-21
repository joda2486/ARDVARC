"""
ARDVARC Flight Software

Author: Timothy Behrer
email: timothy.behrer@colorado.edu

Description:
    constants used by fsw

Notes:
    - 
"""
import sys
import cv2
from cv2 import aruco
import numpy as np
import os
import argparse
import numpy.typing as npt
from enum import IntEnum
from dataclasses import dataclass
import rospy
from typing import Dict, Tuple
from scipy.spatial.transform import Rotation
## TODO Check that all imports are correct


"""System"""
# Possible values of the `measurement_source` field
MEAS_FROM_CAMERA: int = 10
MEAS_FROM_BLUETOOTH: int = 11


"""Pi Camera Parameters"""
## TODO Could be determined from the camera calibration function from cv2.aruco
## TODO This Needs to be updated to the true camera intrinsic parameters
INTRINSICS_PI_CAMERA: npt.NDArray = np.array([[916.6706684, 0, 720], [0, 916.6706684, 540], [0, 0, 1]])
## TODO This Needs to be updated to the true camera distortion parameters
## TODO Could be determined from the camera calibration function from cv2.aruco
DISTORTION: npt.NDArray = np.array([0.0, 0.0, 0.0, 0.0])
## TODO Update the formatting of the camera extrinsic parameters
## TODO Configure a way to get the camera extrinsic parameters accurately
EXTRINSICS_PI_CAMERA_DCM: npt.NDArray = Rotation.from_euler("ZY", (90, 180), degrees=True).as_matrix() #DCM From UAS 2 Camera
EXTRINSICS_PI_CAMERA_TVEC: npt.NDArray = np.array([[0.2], [0.0], [0.0]]) #tvec for Camera from UAS in UAS Frame
## TODO Verify the units on this, I think it needs to match the calibration units (mm)
MARKER_SIZE: float = .352 # meters (15 inches)


"""RGV"""
class RGV_ID(IntEnum):
    RGV1 = 1
    RGV2 = 2
    RGVBOTH = 3
    
ARUCO_ID2RGV_DICT: Dict[Tuple[str, int], RGV_ID] = {
	("DICT_6X6_50", 5): RGV_ID.RGV1,
    ("DICT_APRILTAG_36h11", 5): RGV_ID.RGV2
}
    


"""ArUco"""
# DICTIONARY: cv2.aruco.Dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
#Define the dictionary of ArUco markers Possible
## TODO Comment out the ArUco dictionary that is not being used
## TODO Determine the best ArUco dictionary to use
## TODO Create a new dictionary containing only the two aruco tags used.
ARUCO_DICT = {
    ## Currently Not using the Commented out ones
    
	# "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	# "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	# "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	# "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	# "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	# "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	# "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	# "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	# "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	# "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	# "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	# "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	# "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	# "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	# "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	# "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	# "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	# "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	# "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}
DICTIONARY: cv2.aruco.Dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)

"""State Machine Criteria"""
RECENT_ESTIMATE_TIME_CUTOFF: rospy.Duration = rospy.Duration.from_sec(4)
LOCALIZE_DURATION: rospy.Duration = rospy.Duration.from_sec(90)
JOINT_DURATION: rospy.Duration = rospy.Duration.from_sec(240)
RECENT_SIGHTING_TIME_CUTOFF: rospy.Duration = rospy.Duration.from_sec(2)
MINIMUM_LOCALIZE_DURATION: rospy.Duration = rospy.Duration.from_sec(60)
CONFIDENT_ESTIMATE_THRESHOLD: float = 0.2

"""Guidance"""
# TODO(LF) review this
# ask Rob what this should be for optimal bluetooth measurements 
# Aidan has some numbers that closer is better
# ORBITAL_RADIUS_SINGLE = 1.0 # meters (ground distance)
ORBITAL_RADIUS_SINGLE = 0.5 # meters (ground distance)

ORBITAL_PERIOD = 20  # seconds to complete a full orbit

TIME_AT_ORBIT_POINT = 5  # [seconds] loiter at each point in the orbit for 5 seconds

# TODO(LF) review this
UAS_ALTITUDE_SETPOINT_JOINT = 18.0 # meters (Little under 60 ft)

# TODO(LF) review this
UAS_ALTITUDE_SETPOINT = 9.2 # meters (little over 30 ft)

# Magic Number that's the center of the aerospace backyard in lat/long
# This is decimal lat/long, NOT mins, secs
AERO_BACKYARD_APPROX_CENTER = [40.010886, -105.243878]
AERO_BACKYARD_APPROX_ALT = 1614.001932 # meters


# Initial setpoint at starting location so that pixhawk has setpoints in the buffer
# before we switch into offboard mode
# TODO(LF) review before flight because this will be the first setpoint sent and will also be
# sent in null-type cases
# this specifically is the point in local frame where the pilot is planning on having the drone in hold mode when the pilot switches to offboard mode
CENTER_SETPOINT = [0,0,0]
HOME_SETPOINT = [-10,-10,0]

"""Estimator"""
SPEED_THRESHOLD = 0.3 # m/s

BLUETOOTH_WEIGHT = 1
CAMERA_WEIGHT = 100
IDEAL_TOTAL_WEIGHT = 200

MAX_PLAUSIBLE_RGV_SPEED = 3 # m/s
MISSION_AREA_HALF_WIDTH = 22.86 # m
ESTIMATE_HISTORY_DURATION = 2 # s
MAX_BLIND_FOLLOW_DURATION = 2 # s