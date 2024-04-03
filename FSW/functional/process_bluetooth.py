import rospy
import collections
import math
import numpy as np 

from rosardvarc.msg import UasToRgvDirectionVectorUasFrame, BluetoothAzimuthElevation
from ..config.topic_names import UAS_TO_RGV_DIRECTION_VECTORS, RAW_BLUETOOTH
from ..config.constants import 



def _bluetooth_callback(msg: BluetoothAzimuthElevation):
    # Calculates pointing vector in Bluetooth body frame
    
    """Bluetooth"""
    BLUETOOTH_DCM: npt.NDArray = np.array([[.2682,.8012,-.5350],[.3590,-.5985,-.7162],[-.8940,0,-.4481]])

    msg.timestamp
    if _bluetooth_sub.rgv_id == 1:
        x = math.cos(BluetoothAzimuthElevation.elevation*math.pi/180)*math.cos(BluetoothAzimuthElevation.azimuth*math.pi/180)
        y = math.cos(BluetoothAzimuthElevation.elevation*math.pi/180)*math.cos(BluetoothAzimuthElevation.azimuth*math.pi/180)
        z = math.sin(BluetoothAzimuthElevation.elevation*math.pi/180)

        BluetoothToRgvDirectionVector = [[x,y,z]]
        UASToRgvDirectionVector_bluetooth = BLUETOOTH_DCM*BluetoothToRgvDirectionVector

        x_RGV1_buffer = collections.deque([],100)
        y_RGV1_buffer = collections.deque([],100)
        z_RGV1_buffer = collections.deque([],100)

        x_RGV1_buffer.appendleft(UASToRgvDirectionVector_bluetooth[0])
        y_RGV1_buffer.appendleft(UASToRgvDirectionVector_bluetooth[1])
        z_RGV1_buffer.appendleft(UASToRgvDirectionVector_bluetooth[2])

        UASToRgvDirectionVector = np.array([[x_RGV1_buffer],[y_RGV1_buffer],[z_RGV1_buffer]])


    elif _bluetooth_sub == 2:
        x2 = math.cos(BluetoothAzimuthElevation.elevation*math.pi/180)*math.cos(BluetoothAzimuthElevation.azimuth*math.pi/180)
        y2 = math.cos(BluetoothAzimuthElevation.elevation*math.pi/180)*math.cos(BluetoothAzimuthElevation.azimuth*math.pi/180)
        z2 = math.sin(BluetoothAzimuthElevation.elevation*math.pi/180)

        BluetoothToRgvDirectionVector2 = [[x2,y2,z2]]
        UASToRgvDirectionVector_bluetooth2 = BLUETOOTH_DCM*BluetoothToRgvDirectionVector2

        x_RGV2_buffer = collections.deque([],100)
        y_RGV2_buffer = collections.deque([],100)
        z_RGV2_buffer = collections.deque([],100)

        x_RGV2_buffer.appendleft(UASToRgvDirectionVector_bluetooth2[0])
        y_RGV2_buffer.appendleft(UASToRgvDirectionVector_bluetooth2[1])
        z_RGV2_buffer.appendleft(UASToRgvDirectionVector_bluetooth2[2])

        UASToRgvDirectionVector = np.array([[x_RGV2_buffer],[y_RGV2_buffer],[z_RGV2_buffer]])










    #TODO: convert calculated points to pointing vector (NEED Current attitude of BlueTooth Array in reference to UAS frame)

    #TODO: Create storage array that stores values (Might not need if Bluetooth frequency is 20)


    # Rotation Matrix Call goes here Euler Angles = [180,90,0]
    
    rospy.logdebug("Bluetooth processor received raw bluetooth data")
    rospy.logdebug("Bluetooth processor published a direction vector")
    
    # Probably will use publisher below because I need to send beacon information
    
    # publisher below seems much, unless someone had a syntax they wanted
    _direction_vector_pub.publish(
        UasToRgvDirectionVectorUasFrame(
            # TODO: Make this something reasonable
            timestamp = _bluetooth_sub.timestamp,
            rgv_id = _bluetooth_sub.rgv_id,
            measurement_source = 11,
            direction = UASToRgvDirectionVector
        )
    )


def setup():
    """
    Setup publishers and subscribers for process_bluetooth.py
    """
    
    global _direction_vector_pub, _bluetooth_sub
    
    _direction_vector_pub = rospy.Publisher(UAS_TO_RGV_DIRECTION_VECTORS, UasToRgvDirectionVectorUasFrame, queue_size=1)
    _bluetooth_sub = rospy.Subscriber(RAW_BLUETOOTH, BluetoothAzimuthElevation, _bluetooth_callback)


_direction_vector_pub: rospy.Publisher
_bluetooth_sub: rospy.Subscriber