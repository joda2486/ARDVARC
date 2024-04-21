import rospy
from rosardvarc.msg import UasToRgvDirectionVectorUasFrame, BluetoothAzimuthElevation
from ..config.topic_names import UAS_TO_RGV_DIRECTION_VECTORS, RAW_BLUETOOTH


def _bluetooth_callback(msg: BluetoothAzimuthElevation):
    # math math math
    
    rospy.logdebug("Bluetooth processor received raw bluetooth data")
    rospy.logdebug("Bluetooth processor published a direction vector")
    # _direction_vector_pub.publish(
    #     UasToRgvDirectionVectorUasFrame(
    #         # TODO: Make this something reasonable
    #     )
    # )


def setup():
    """
    Setup publishers and subscribers for process_bluetooth.py
    """
    
    global _direction_vector_pub, _bluetooth_sub
    
    _direction_vector_pub = rospy.Publisher(UAS_TO_RGV_DIRECTION_VECTORS, UasToRgvDirectionVectorUasFrame, queue_size=1)
    _bluetooth_sub = rospy.Subscriber(RAW_BLUETOOTH, BluetoothAzimuthElevation, _bluetooth_callback)


_direction_vector_pub: rospy.Publisher
_bluetooth_sub: rospy.Subscriber