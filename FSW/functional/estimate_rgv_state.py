from dataclasses import dataclass
import rospy
from ..config.topic_names import ESTIMATED_RGV_STATES, RGV_PROJECTIONS
from..config.constants import RGV_ID, MEAS_FROM_BLUETOOTH, MEAS_FROM_CAMERA, SPEED_THRESHOLD, \
    BLUETOOTH_WEIGHT, CAMERA_WEIGHT, MAX_PLAUSIBLE_RGV_SPEED, MISSION_AREA_HALF_WIDTH, \
    ORBITAL_RADIUS_SINGLE, ESTIMATE_HISTORY_DURATION, MAX_BLIND_FOLLOW_DURATION, \
    IDEAL_TOTAL_WEIGHT
from rosardvarc.msg import RgvLocalProjection, EstimatedRgvState
from typing import Tuple
import numpy as np
from ..util.sorted_buffer import SortedBuffer


@dataclass
class SingleRgvEstimate:
    confidence: float
    moving: bool
    x_slope: float
    x_intercept: float
    y_slope: float
    y_intercept: float
    timestamp: rospy.Time


def _projection_key_func(rgv_projection: RgvLocalProjection) -> float:
    return rgv_projection.timestamp.to_sec()


_rgv_1_camera_projection_buffer = SortedBuffer(_projection_key_func)
_rgv_1_bluetooth_projection_buffer = SortedBuffer(_projection_key_func)
_new_rgv_1_projection = False
_rgv_2_camera_projection_buffer = SortedBuffer(_projection_key_func)
_rgv_2_bluetooth_projection_buffer = SortedBuffer(_projection_key_func)
_new_rgv_2_projection = False

_rgv_1_estimate = SingleRgvEstimate(0, True, 0, 0, 0, 0, rospy.Time.now())
_rgv_2_estimate = SingleRgvEstimate(0, True, 0, 0, 0, 0, rospy.Time.now())

_estimated_rgv_state_pub: rospy.Publisher
_rgv_projection_sub: rospy.Subscriber


def _build_estimate_msg(rgv1_estimate: SingleRgvEstimate, rgv2_estimate: SingleRgvEstimate) -> EstimatedRgvState:
    msg = EstimatedRgvState()
    
    msg.timestamp = rospy.Time.now()
    
    msg.rgv1_confidence = rgv1_estimate.confidence
    msg.rgv1_moving = rgv1_estimate.moving
    msg.rgv1_position_local = _get_position_from_estimate(rgv1_estimate)
    
    msg.rgv2_confidence = rgv2_estimate.confidence
    msg.rgv2_moving = rgv2_estimate.moving
    msg.rgv2_position_local = _get_position_from_estimate(rgv2_estimate)
    
    return msg


def _rgv_projection_callback(msg: RgvLocalProjection):
    global _new_rgv_1_projection, _new_rgv_2_projection
    
    if (msg.rgv_id, msg.measurement_source) == (RGV_ID.RGV1, MEAS_FROM_CAMERA):
        _rgv_1_camera_projection_buffer.add(msg)
        _new_rgv_1_projection = True
    elif (msg.rgv_id, msg.measurement_source) == (RGV_ID.RGV1, MEAS_FROM_BLUETOOTH):
        _rgv_1_bluetooth_projection_buffer.add(msg)
        _new_rgv_1_projection = True
    elif (msg.rgv_id, msg.measurement_source) == (RGV_ID.RGV2, MEAS_FROM_CAMERA):
        _rgv_2_camera_projection_buffer.add(msg)
        _new_rgv_2_projection = True
    elif (msg.rgv_id, msg.measurement_source) == (RGV_ID.RGV2, MEAS_FROM_BLUETOOTH):
        _rgv_2_bluetooth_projection_buffer.add(msg)
        _new_rgv_2_projection = True
    else:
        raise Exception(f"Unrecognized projection from measurement source {msg.measurement_source} for ID {msg.rgv_id}.")
    rospy.logdebug("RGV state estimator saved projection")


def setup():
    """
    Setup publishers and subscribers for estimate_rgv_state.py
    """

    global _estimated_rgv_state_pub, _rgv_projection_sub
    _estimated_rgv_state_pub = rospy.Publisher(ESTIMATED_RGV_STATES, EstimatedRgvState, queue_size=1)
    _rgv_projection_sub = rospy.Subscriber(RGV_PROJECTIONS, RgvLocalProjection, _rgv_projection_callback)


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


def _get_position_from_estimate(estimate: SingleRgvEstimate) -> Tuple[float, float, float]:
    now = rospy.Time.now().to_sec()
    if (now - estimate.timestamp.to_sec()) > MAX_BLIND_FOLLOW_DURATION:
        now = estimate.timestamp.to_sec() + MAX_BLIND_FOLLOW_DURATION
    
    x_now = estimate.x_slope * now + estimate.x_intercept
    y_now = estimate.y_slope * now + estimate.y_intercept
    
    # Clamp estimate to within mission area
    if x_now > MISSION_AREA_HALF_WIDTH - ORBITAL_RADIUS_SINGLE:
        x_now = MISSION_AREA_HALF_WIDTH - ORBITAL_RADIUS_SINGLE
    elif x_now < ORBITAL_RADIUS_SINGLE - MISSION_AREA_HALF_WIDTH:
        x_now = ORBITAL_RADIUS_SINGLE - MISSION_AREA_HALF_WIDTH
    if y_now > MISSION_AREA_HALF_WIDTH - ORBITAL_RADIUS_SINGLE:
        y_now = MISSION_AREA_HALF_WIDTH - ORBITAL_RADIUS_SINGLE
    elif y_now < ORBITAL_RADIUS_SINGLE - MISSION_AREA_HALF_WIDTH:
        y_now = ORBITAL_RADIUS_SINGLE - MISSION_AREA_HALF_WIDTH
    
    return (x_now, y_now, 0)


def _estimate_rgv_state() -> EstimatedRgvState:
    """
    Estimates the RGV state.
    """
    
    global _rgv_1_estimate, _rgv_2_estimate, _new_rgv_1_projection, _new_rgv_2_projection
    
    # Remove old projections
    old_threshold_time = rospy.Time.now().to_sec() - ESTIMATE_HISTORY_DURATION
    _remove_projections_before(old_threshold_time, _rgv_1_bluetooth_projection_buffer)
    _remove_projections_before(old_threshold_time, _rgv_2_bluetooth_projection_buffer)
    _remove_projections_before(old_threshold_time, _rgv_1_camera_projection_buffer)
    _remove_projections_before(old_threshold_time, _rgv_2_camera_projection_buffer)
    
    # Build estimates by fitting a line
    _update_estimate(_rgv_1_estimate, _rgv_1_bluetooth_projection_buffer, _rgv_1_camera_projection_buffer, _new_rgv_1_projection)
    _update_estimate(_rgv_2_estimate, _rgv_2_bluetooth_projection_buffer, _rgv_2_camera_projection_buffer, _new_rgv_2_projection)
    _new_rgv_1_projection = False
    _new_rgv_2_projection = False
    
    return _build_estimate_msg(_rgv_1_estimate, _rgv_2_estimate)


def _remove_projections_before(threshold_time: float, buffer: SortedBuffer[RgvLocalProjection]):
    index = buffer.bisect_key_left(threshold_time)
    del buffer[:index]


def _update_estimate(rgv_estimate: SingleRgvEstimate,
                     bluetooth_buffer: SortedBuffer[RgvLocalProjection],
                     camera_buffer: SortedBuffer[RgvLocalProjection],
                     new_rgv_projection: bool):
    # Calculate confidence based on number of data points
    rgv_estimate.confidence = (BLUETOOTH_WEIGHT * len(bluetooth_buffer) + CAMERA_WEIGHT * len(camera_buffer))/IDEAL_TOTAL_WEIGHT
    if rgv_estimate.confidence > 1:
        rgv_estimate.confidence = 1
    
    # If there's no new data then just use the old estimate with the new confidence
    if not new_rgv_projection:
        rgv_estimate.moving = True
        return
        
    bluetooth_times = [proj.timestamp.to_sec() for proj in bluetooth_buffer]
    bluetooth_xs = [proj.rgv_position_local[0] for proj in bluetooth_buffer]
    bluetooth_ys = [proj.rgv_position_local[1] for proj in bluetooth_buffer]
    camera_times = [proj.timestamp.to_sec() for proj in camera_buffer]
    camera_xs = [proj.rgv_position_local[0] for proj in camera_buffer]
    camera_ys = [proj.rgv_position_local[1] for proj in camera_buffer]
    
    all_times = bluetooth_times + camera_times
    all_xs = bluetooth_xs + camera_xs
    all_ys = bluetooth_ys + camera_ys
    
    # If there is only one projection, use it as the estimate
    if len(all_xs) == 1:
        rgv_estimate.x_intercept = all_xs[0]
        rgv_estimate.y_intercept = all_ys[0]
        rgv_estimate.x_slope = 0
        rgv_estimate.y_slope = 0
        rgv_estimate.moving = True
        rgv_estimate.timestamp = rospy.Time.now()
        return
    
    weights = [BLUETOOTH_WEIGHT] * len(bluetooth_buffer) + [CAMERA_WEIGHT] * len(camera_buffer)
    
    rgv_estimate.x_slope, rgv_estimate.x_intercept = np.polyfit(all_times, all_xs, 1, w=weights)
    rgv_estimate.y_slope, rgv_estimate.y_intercept = np.polyfit(all_times, all_ys, 1, w=weights)
    
    speed = np.sqrt(rgv_estimate.x_slope**2+rgv_estimate.y_slope**2)
    
    rgv_estimate.moving = speed > SPEED_THRESHOLD
    
    if speed > MAX_PLAUSIBLE_RGV_SPEED:
        # This is clearly a bad estimate, let's just average instead
        rgv_estimate.x_slope = 0
        rgv_estimate.y_slope = 0
        rgv_estimate.x_intercept = float(np.mean(all_xs))
        rgv_estimate.y_intercept = float(np.mean(all_ys))
    
    rgv_estimate.timestamp = rospy.Time.now()