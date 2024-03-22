from dataclasses import dataclass
import rospy
import collections
from ..config.topic_names import ESTIMATED_RGV_STATES, UAS_POSES, UAS_TO_RGV_DIRECTION_VECTORS
from..config.constants import RGV_ID, MEAS_FROM_BLUETOOTH, MEAS_FROM_CAMERA, SPEED_THRESHOLD
from rosardvarc.msg import EstimatedRgvState, UasToRgvDirectionVectorUasFrame
from geometry_msgs.msg import PoseStamped
from typing import Iterable, overload, Generic, Tuple, Deque, List, Optional, Callable, Any, TypeVar, Union
import numpy as np
from scipy.spatial.transform import Rotation
from sortedcontainers import SortedKeyList
from tf.transformations import quaternion_slerp


@dataclass
class SingleRgvEstimate:
    confidence: float = 0
    moving: bool = True
    position_local: Tuple[float, float, float] = (0, 0, 0)
    timestamp: rospy.Time = rospy.Time.now()

T = TypeVar('T')

class SortedBuffer(SortedKeyList, Generic[T]):
    def __init__(self, key: Callable[[T], float], iterable: Union[Iterable[T], None] = None, capacity: int = 50):
        super().__init__(iterable, key)
        self.capacity = capacity
    
    def add(self, value: T):
        super().add(value)
        if len(self) > self.capacity:
            self.pop(0)
    
    # https://stackoverflow.com/questions/71182977/what-is-the-type-annotation-for-return-value-of-getitem
    @overload
    def __getitem__(self, index: slice) -> List[T]: ...
    
    @overload
    def __getitem__(self, index: int) -> T: ...
    
    def __getitem__(self, index: Union[slice, int]) -> Union[T, List[T]]:
        return super().__getitem__(index)

def _uas_state_key_func(uas_state: PoseStamped) -> float:
    return uas_state.header.stamp.to_sec()

def _direction_vector_key_func(direction_vector: UasToRgvDirectionVectorUasFrame) -> float:
    return direction_vector.timestamp.to_sec()

def _single_rgv_estimate_key_func(single_rgv_estimate: SingleRgvEstimate) -> float:
    return single_rgv_estimate.timestamp.to_sec()

_uas_states = SortedBuffer(_uas_state_key_func)
_rgv_1_camera_direction_vector_buffer = SortedBuffer(_direction_vector_key_func)
_rgv_1_bluetooth_direction_vector_buffer = SortedBuffer(_direction_vector_key_func)
_rgv_2_camera_direction_vector_buffer = SortedBuffer(_direction_vector_key_func)
_rgv_2_bluetooth_direction_vector_buffer = SortedBuffer(_direction_vector_key_func)
_rgv_1_position_buffer = SortedBuffer(_single_rgv_estimate_key_func)
_rgv_2_position_buffer = SortedBuffer(_single_rgv_estimate_key_func)

_estimated_rgv_state_pub: rospy.Publisher
_uas_state_sub: rospy.Subscriber
_direction_vector_sub: rospy.Subscriber


def _build_estimate_msg(rgv1_estimate: SingleRgvEstimate = SingleRgvEstimate(), rgv2_estimate: SingleRgvEstimate = SingleRgvEstimate()) -> EstimatedRgvState:
    msg = EstimatedRgvState()
    
    msg.timestamp = rospy.Time.now()
    
    msg.rgv1_confidence = rgv1_estimate.confidence
    msg.rgv1_moving = rgv1_estimate.moving
    msg.rgv1_position_local = rgv1_estimate.position_local
    
    msg.rgv2_confidence = rgv2_estimate.confidence
    msg.rgv2_moving = rgv2_estimate.moving
    msg.rgv2_position_local = rgv2_estimate.position_local
    
    return msg


def _uas_state_callback(msg: PoseStamped):
    _uas_states.add(msg)
    rospy.logdebug("RGV state estimator saved UAS state")


def _direction_vector_callback(msg: UasToRgvDirectionVectorUasFrame):
    if (msg.rgv_id, msg.measurement_source) == (RGV_ID.RGV1, MEAS_FROM_CAMERA):
        _rgv_1_camera_direction_vector_buffer.add(msg)
    elif (msg.rgv_id, msg.measurement_source) == (RGV_ID.RGV1, MEAS_FROM_BLUETOOTH):
        _rgv_1_bluetooth_direction_vector_buffer.add(msg)
    elif (msg.rgv_id, msg.measurement_source) == (RGV_ID.RGV2, MEAS_FROM_CAMERA):
        _rgv_2_camera_direction_vector_buffer.add(msg)
    elif (msg.rgv_id, msg.measurement_source) == (RGV_ID.RGV2, MEAS_FROM_BLUETOOTH):
        _rgv_2_bluetooth_direction_vector_buffer.add(msg)
    else:
        raise Exception(f"Unrecognized direction vector from measurement source {msg.measurement_source} for ID {msg.rgv_id}.")
    rospy.logdebug("RGV state estimator saved direction vector")


def setup():
    """
    Setup publishers and subscribers for estimate_rgv_state.py
    """

    global _estimated_rgv_state_pub, _uas_state_sub, _direction_vector_sub
    _estimated_rgv_state_pub = rospy.Publisher(ESTIMATED_RGV_STATES, EstimatedRgvState, queue_size=1)
    _uas_state_sub = rospy.Subscriber(UAS_POSES, PoseStamped, _uas_state_callback)
    _direction_vector_sub = rospy.Subscriber(UAS_TO_RGV_DIRECTION_VECTORS, UasToRgvDirectionVectorUasFrame, _direction_vector_callback)


def publish_estimated_rgv_state():
    """
    Called by main.py in the main loop.
    Estimates and publishes the RGV state.
    """
    # Estimate the RGV state
    msg = _estimate_rgv_state()
    
    # Publish estimate
    rospy.logdebug("RGV state estimator published an RGV state estimate")
    _estimated_rgv_state_pub.publish(msg)


def _estimate_rgv_state() -> EstimatedRgvState:
    """
    Estimates the RGV state.
    """
    # Estimate the RGV state
    
    # TODO: This is a very naive MVP, we should do smarter math eventually
    
    rgv1_estimate = _build_single_estimate(_rgv_1_bluetooth_direction_vector_buffer, _rgv_1_camera_direction_vector_buffer, _uas_states, _rgv_1_position_buffer)
    rgv2_estimate = _build_single_estimate(_rgv_2_bluetooth_direction_vector_buffer, _rgv_2_camera_direction_vector_buffer, _uas_states, _rgv_2_position_buffer)
    
    return _build_estimate_msg(rgv1_estimate, rgv2_estimate)


def _build_single_estimate(bluetooth_buffer: SortedBuffer[UasToRgvDirectionVectorUasFrame],
                           camera_buffer: SortedBuffer[UasToRgvDirectionVectorUasFrame],
                           uas_buffer: SortedBuffer[PoseStamped],
                           recent_positions: SortedBuffer[SingleRgvEstimate]) -> SingleRgvEstimate:
    # If we don't have any UAS readings, give up
    if len(uas_buffer) == 0:
        return SingleRgvEstimate()
    
    # Get most recent bluetooth measurement so long as it is from within the last second
    if len(bluetooth_buffer) > 0 and (rospy.Time.now() - bluetooth_buffer[-1].timestamp).to_sec() < 1:
        most_recent_bluetooth = bluetooth_buffer[-1]
    else:
        most_recent_bluetooth = None
    
    # Get most recent camera measurement so long as it is from within the last second
    if len(camera_buffer) > 0 and (rospy.Time.now() - camera_buffer[-1].timestamp).to_sec() < 1:
        most_recent_camera = camera_buffer[-1]
    else:
        most_recent_camera = None
    
    # If we don't have either, return either a recent estimate or the default low-confidence
    if most_recent_bluetooth is None and most_recent_camera is None:
        if len(recent_positions) == 0 or (rospy.Time.now() - recent_positions[-1].timestamp).to_sec() > 5:
            return SingleRgvEstimate()
        return SingleRgvEstimate(position_local=recent_positions[-1].position_local)
    
    # Determine what our best pointing vector is and how confident we are in it
    if most_recent_bluetooth is not None and most_recent_camera is None:
        # If we only have bluetooth, use that as the pointing vector and set the confidence low
        best_pointing_vector = np.array(most_recent_bluetooth.direction)
        best_pointing_vector_time = most_recent_bluetooth.timestamp.to_sec()
        confidence = 0.2
    elif most_recent_bluetooth is None and most_recent_camera is not None:
        # If we only have camera (somehow), use that as the pointing vector and set the confidence medium
        best_pointing_vector = np.array(most_recent_camera.direction)
        best_pointing_vector_time = most_recent_camera.timestamp.to_sec()
        confidence = 0.6
    elif most_recent_bluetooth is not None and most_recent_camera is not None:
        # If we have both, use a weighted average as the pointing vector and set the confidence high
        best_pointing_vector = np.array(most_recent_camera.direction) * 0.8 + np.array(most_recent_bluetooth.direction) * 0.2
        best_pointing_vector_time = most_recent_camera.timestamp.to_sec() * 0.8 + most_recent_bluetooth.timestamp.to_sec() * 0.2
        confidence = 1
        
    # Get the UAS pose that best aligns with the time of the best pointing vector
    previous_uas_pose_index = uas_buffer.bisect_key_left(best_pointing_vector_time)
    
    if previous_uas_pose_index + 1 == len(uas_buffer):
        # We don't have a UAS pose that is more recent than the best pointing vector, so just use the most recent one
        best_uas_pose = uas_buffer[previous_uas_pose_index]
        best_uas_orientation = np.array([best_uas_pose.pose.orientation.x, best_uas_pose.pose.orientation.y, best_uas_pose.pose.orientation.z, best_uas_pose.pose.orientation.w])
        best_uas_position_inertial = np.array([best_uas_pose.pose.position.x, best_uas_pose.pose.position.y, best_uas_pose.pose.position.z])
    else:
        # Linearly interpolate between UAS poses
        previous_uas_pose = uas_buffer[previous_uas_pose_index]
        previous_uas_pose_time = previous_uas_pose.header.stamp.to_sec()
        previous_uas_orientation = np.array([previous_uas_pose.pose.orientation.x, previous_uas_pose.pose.orientation.y, previous_uas_pose.pose.orientation.z, previous_uas_pose.pose.orientation.w])
        previous_uas_position_inertial = np.array([previous_uas_pose.pose.position.x, previous_uas_pose.pose.position.y, previous_uas_pose.pose.position.z])
        next_uas_pose = uas_buffer[previous_uas_pose_index + 1]
        next_uas_pose_time = next_uas_pose.header.stamp.to_sec()
        next_uas_orientation = np.array([next_uas_pose.pose.orientation.x, next_uas_pose.pose.orientation.y, next_uas_pose.pose.orientation.z, next_uas_pose.pose.orientation.w])
        next_uas_position_inertial = np.array([next_uas_pose.pose.position.x, next_uas_pose.pose.position.y, next_uas_pose.pose.position.z])
        previous_interpolation_weight = (next_uas_pose_time-best_pointing_vector_time)/(next_uas_pose_time-previous_uas_pose_time)
        next_interpolation_weight = 1 - previous_interpolation_weight
        best_uas_orientation = quaternion_slerp(previous_uas_orientation, next_uas_orientation, next_interpolation_weight)
        best_uas_position_inertial = previous_uas_position_inertial * previous_interpolation_weight + next_uas_position_inertial * next_interpolation_weight
        
    # if previous_uas_pose_index == len(uas_buffer):
    #     best_uas_pose = uas_buffer[previous_uas_pose_index-1]
    # else:
    #     best_uas_pose = uas_buffer[previous_uas_pose_index]
    # print(best_pointing_vector_time - best_uas_pose.header.stamp.to_sec())
    # best_uas_orientation = best_uas_pose.pose.orientation
    # best_uas_position = best_uas_pose.pose.position
    
    # Find where the best pointing vector intersects the ground
    uas_rotation = Rotation.from_quat(best_uas_orientation)
    best_pointing_vector_inertial = uas_rotation.apply(best_pointing_vector, inverse=True)
    coefficient = -best_uas_position_inertial[2]/best_pointing_vector_inertial[2]
    rgv_position_inertial = best_uas_position_inertial + coefficient * best_pointing_vector_inertial
    
    # Determine if it is moving by comparing to the recent average
    if len(recent_positions) < 5:
        # Assume it is moving if we don't know very much about it
        moving = True
    else:
        recent_position_avg = np.mean(np.array([recent_position.position_local for recent_position in recent_positions]), axis=0)
        recent_time_avg = np.mean([recent_position.timestamp.to_sec() for recent_position in recent_positions])
        rough_speed = float(np.linalg.norm(rgv_position_inertial-recent_position_avg)/(rospy.Time.now().to_sec()-recent_time_avg))
        moving = rough_speed > SPEED_THRESHOLD
    
    # Build the return value
    ret = SingleRgvEstimate(confidence, moving, rgv_position_inertial, rospy.Time.now())
    
    # Update the recent positions deque
    recent_positions.add(ret)
        
    # Return the return value
    return ret