import rospy
from ..config.topic_names import RGV_PROJECTIONS, UAS_POSES, UAS_TO_RGV_DIRECTION_VECTORS
from rosardvarc.msg import RgvLocalProjection, UasToRgvDirectionVectorUasFrame
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation
from ..util.sorted_buffer import SortedBuffer

def _uas_state_key_func(uas_state: PoseStamped) -> float:
    return uas_state.header.stamp.to_sec()

_uas_states = SortedBuffer(_uas_state_key_func)

_projected_rgv_state_pub: rospy.Publisher
_uas_state_sub: rospy.Subscriber
_direction_vector_sub: rospy.Subscriber


def _uas_state_callback(msg: PoseStamped):
    _uas_states.add(msg)
    rospy.logdebug("RGV state estimator saved UAS state")


def _direction_vector_callback(msg: UasToRgvDirectionVectorUasFrame):
    # If we don't have enough UAS readings to interpolate, give up
    if len(_uas_states) < 2:
        return
        
    # Get the UAS pose that best aligns with the time of the pointing vector
    time = msg.timestamp.to_sec()
    next_uas_pose_index = _uas_states.bisect_key_left(time)
    
    if next_uas_pose_index == 0:
        next_uas_pose_index = 1
    elif next_uas_pose_index == len(_uas_states):
        next_uas_pose_index -= 1
    
    previous_uas_pose_index = next_uas_pose_index - 1
    
    # Linearly interpolate between UAS poses
    previous_uas_pose = _uas_states[previous_uas_pose_index]
    previous_uas_pose_time = previous_uas_pose.header.stamp.to_sec()
    previous_uas_orientation = np.array([previous_uas_pose.pose.orientation.x, previous_uas_pose.pose.orientation.y, previous_uas_pose.pose.orientation.z, previous_uas_pose.pose.orientation.w])
    previous_uas_position_inertial = np.array([previous_uas_pose.pose.position.x, previous_uas_pose.pose.position.y, previous_uas_pose.pose.position.z])
    next_uas_pose = _uas_states[next_uas_pose_index]
    next_uas_pose_time = next_uas_pose.header.stamp.to_sec()
    next_uas_orientation = np.array([next_uas_pose.pose.orientation.x, next_uas_pose.pose.orientation.y, next_uas_pose.pose.orientation.z, next_uas_pose.pose.orientation.w])
    next_uas_position_inertial = np.array([next_uas_pose.pose.position.x, next_uas_pose.pose.position.y, next_uas_pose.pose.position.z])
    previous_interpolation_weight = (next_uas_pose_time-time)/(next_uas_pose_time-previous_uas_pose_time)
    next_interpolation_weight = 1 - previous_interpolation_weight
    best_uas_orientation = previous_uas_orientation * previous_interpolation_weight + next_uas_orientation * next_interpolation_weight
    best_uas_position_inertial = previous_uas_position_inertial * previous_interpolation_weight + next_uas_position_inertial * next_interpolation_weight
    
    # Find where the best pointing vector intersects the ground
    uas_rotation = Rotation.from_quat(best_uas_orientation)
    best_pointing_vector_inertial = uas_rotation.apply(np.array(msg.direction))
    coefficient = -best_uas_position_inertial[2]/best_pointing_vector_inertial[2]
    rgv_position_inertial = best_uas_position_inertial + coefficient * best_pointing_vector_inertial

    # Build return message
    ret_msg = RgvLocalProjection()
    ret_msg.measurement_source = msg.measurement_source
    ret_msg.rgv_id = msg.rgv_id
    ret_msg.timestamp = msg.timestamp
    ret_msg.rgv_position_local = rgv_position_inertial[0:2]
        
    # Publish it
    _projected_rgv_state_pub.publish(ret_msg)


def setup():
    """
    Setup publishers and subscribers for projector.py
    """

    global _projected_rgv_state_pub, _uas_state_sub, _direction_vector_sub
    _projected_rgv_state_pub = rospy.Publisher(RGV_PROJECTIONS, RgvLocalProjection, queue_size=1)
    _uas_state_sub = rospy.Subscriber(UAS_POSES, PoseStamped, _uas_state_callback)
    _direction_vector_sub = rospy.Subscriber(UAS_TO_RGV_DIRECTION_VECTORS, UasToRgvDirectionVectorUasFrame, _direction_vector_callback)