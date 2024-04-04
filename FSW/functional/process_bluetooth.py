import rospy
import collections
import math
import numpy as np 

from rosardvarc.msg import UasToRgvDirectionVectorUasFrame, BluetoothAzimuthElevation
from ..config.topic_names import UAS_TO_RGV_DIRECTION_VECTORS, RAW_BLUETOOTH
from ..config.constants import BLUETOOTH_DCM, RGV_ID, MEAS_FROM_BLUETOOTH


x_RGV1_buffer = collections.deque([],100)
y_RGV1_buffer = collections.deque([],100)
z_RGV1_buffer = collections.deque([],100)
x_RGV2_buffer = collections.deque([],100)
y_RGV2_buffer = collections.deque([],100)
z_RGV2_buffer = collections.deque([],100)


def _bluetooth_callback(msg: BluetoothAzimuthElevation):
    # Calculates pointing vector in Bluetooth body frame
    
    # TODO: Fix math to match how angles are defined (once we figure that out)
    x = math.cos(msg.elevation*math.pi/180)*math.cos(msg.azimuth*math.pi/180)
    y = math.cos(msg.elevation*math.pi/180)*math.cos(msg.azimuth*math.pi/180)
    z = math.sin(msg.elevation*math.pi/180)
    
    # TODO: Reject readings with that give x, y, z values very different from 
    
    BluetoothToRgvDirectionVector = np.array([x,y,z])
    UASToRgvDirectionVector_bluetooth = BLUETOOTH_DCM*BluetoothToRgvDirectionVector

    if msg.rgv_id == RGV_ID.RGV1:
        x_RGV1_buffer.appendleft(UASToRgvDirectionVector_bluetooth[0])
        y_RGV1_buffer.appendleft(UASToRgvDirectionVector_bluetooth[1])
        z_RGV1_buffer.appendleft(UASToRgvDirectionVector_bluetooth[2])

    elif msg.rgv_id == RGV_ID.RGV2:
        x_RGV2_buffer.appendleft(UASToRgvDirectionVector_bluetooth[0])
        y_RGV2_buffer.appendleft(UASToRgvDirectionVector_bluetooth[1])
        z_RGV2_buffer.appendleft(UASToRgvDirectionVector_bluetooth[2])
    
    rospy.logdebug("Bluetooth processor received raw bluetooth data")
    rospy.logdebug("Bluetooth processor published a direction vector")
    
    pub_msg = UasToRgvDirectionVectorUasFrame()
    
    pub_msg.timestamp = msg.timestamp
    pub_msg.rgv_id = msg.rgv_id
    pub_msg.measurement_source = MEAS_FROM_BLUETOOTH
    pub_msg.direction = UASToRgvDirectionVector_bluetooth
    
    _direction_vector_pub.publish(pub_msg)


def setup():
    """
    Setup publishers and subscribers for process_bluetooth.py
    """
    
    global _direction_vector_pub, _bluetooth_sub
    
    _direction_vector_pub = rospy.Publisher(UAS_TO_RGV_DIRECTION_VECTORS, UasToRgvDirectionVectorUasFrame, queue_size=1)
    _bluetooth_sub = rospy.Subscriber(RAW_BLUETOOTH, BluetoothAzimuthElevation, _bluetooth_callback)


_direction_vector_pub: rospy.Publisher
_bluetooth_sub: rospy.Subscriber