from typing import Optional
import rospy
import genpy
from ..config.structures import MissionStates
from ..config.topic_names import STATE_MACHINE_CRITERIA, ESTIMATED_RGV_STATES, RECENT_RGV_SIGHTINGS, MISSION_STATES, BATTERY
from ..config.constants import RGV_ID, RECENT_ESTIMATE_TIME_CUTOFF, LOCALIZE_DURATION, JOINT_DURATION, RECENT_SIGHTING_TIME_CUTOFF, BATTERY_LOW_CHARGE_PCT_CUTOFF, MINIMUM_LOCALIZE_DURATION, CONFIDENT_ESTIMATE_THRESHOLD
from rosardvarc.msg import StateMachineCriteria, EstimatedRgvState, RecentSighting, MissionState
from sensor_msgs.msg import BatteryState
import numpy as np


_time_of_most_recent_rgv_1_sighting: Optional[genpy.Time] = None
_time_of_most_recent_rgv_2_sighting: Optional[genpy.Time] = None

_current_mission_state: MissionStates = MissionStates.FIND_RGV_1
_current_mission_state_start_time: genpy.Time

_time_of_most_recent_confident_rgv_1_estimate: Optional[genpy.Time] = None
_time_of_most_recent_confident_rgv_2_estimate: Optional[genpy.Time] = None
_rgv_1_moving: bool = True
_rgv_2_moving: bool = True

_low_battery: bool = False


def _get_current_mission_state_time_spent(now: rospy.Time) -> rospy.Duration:
    return now - _current_mission_state_start_time
    

def _estimated_rgv_state_callback(msg: EstimatedRgvState):
    """
    This is the callback for the RGV state estimation subscriber. It updates the RGV-state
    globals (recent_confident_estimate and moving) and then constructs and publishes a
    StateMachineCriteria message.
    """
    
    global _time_of_most_recent_confident_rgv_1_estimate, _time_of_most_recent_confident_rgv_2_estimate, _rgv_1_moving, _rgv_2_moving
    
    # Update time-of-estimate globals
    if msg.rgv1_confidence >= CONFIDENT_ESTIMATE_THRESHOLD:
        _time_of_most_recent_confident_rgv_1_estimate = msg.timestamp
        _rgv_1_moving = msg.rgv1_moving
    if msg.rgv2_confidence >= CONFIDENT_ESTIMATE_THRESHOLD:
        _time_of_most_recent_confident_rgv_2_estimate = msg.timestamp
        _rgv_2_moving = msg.rgv2_moving
    
    # Determine state machine criteria
    state_machine_criteria = _build_state_machine_criteria_message(rospy.Time.now())
    
    # Publish state machine criteria
    rospy.logdebug("State machine criteria generator published mission state criteria")
    _state_machine_criteria_pub.publish(state_machine_criteria)


def _build_state_machine_criteria_message(now: rospy.Time) -> StateMachineCriteria:
    """
    Generates a StateMachineCriteria message based off of the various global variables in
    this module.
    """
    
    current_mission_state_time_spent = _get_current_mission_state_time_spent(now)
    
    state_machine_criteria = StateMachineCriteria()
    state_machine_criteria.timestamp = now
    state_machine_criteria.recent_rgv_1_estimate = _time_of_most_recent_confident_rgv_1_estimate is not None and now - _time_of_most_recent_confident_rgv_1_estimate <= RECENT_ESTIMATE_TIME_CUTOFF
    state_machine_criteria.recent_rgv_2_estimate = _time_of_most_recent_confident_rgv_2_estimate is not None and now - _time_of_most_recent_confident_rgv_2_estimate <= RECENT_ESTIMATE_TIME_CUTOFF
    state_machine_criteria.rgv_1_is_moving = _rgv_1_moving
    state_machine_criteria.rgv_2_is_moving = _rgv_2_moving
    state_machine_criteria.rgv_1_localized = _current_mission_state is MissionStates.LOCALIZE_RGV_1 and current_mission_state_time_spent >= LOCALIZE_DURATION
    state_machine_criteria.rgv_2_localized = _current_mission_state is MissionStates.LOCALIZE_RGV_2 and current_mission_state_time_spent >= LOCALIZE_DURATION
    state_machine_criteria.joint_localized = _current_mission_state is MissionStates.JOINT_LOCALIZE and current_mission_state_time_spent >= JOINT_DURATION
    state_machine_criteria.rgv_1_sighted = _time_of_most_recent_rgv_1_sighting is not None and now - _time_of_most_recent_rgv_1_sighting <= RECENT_SIGHTING_TIME_CUTOFF
    state_machine_criteria.rgv_2_sighted = _time_of_most_recent_rgv_2_sighting is not None and now - _time_of_most_recent_rgv_2_sighting <= RECENT_SIGHTING_TIME_CUTOFF
    state_machine_criteria.minimum_localize_time_reached = (_current_mission_state is MissionStates.LOCALIZE_RGV_1 or _current_mission_state is MissionStates.LOCALIZE_RGV_2) and current_mission_state_time_spent >= MINIMUM_LOCALIZE_DURATION
    state_machine_criteria.battery_low = _low_battery
    return state_machine_criteria


def _sightings_callback(msg: RecentSighting):
    """
    This is the callback used by the recent sightings subscriber. It updates the values
    of _time_of_most_recent_rgv_1_sighting and _time_of_most_recent_rgv_2_sighting if an
    RGV was seen.
    """
    
    global _time_of_most_recent_rgv_1_sighting, _time_of_most_recent_rgv_2_sighting
    
    rospy.logdebug("State machine criteria generator saved a recent sighting")
    if msg.rgv_id == RGV_ID.RGV1:
        _time_of_most_recent_rgv_1_sighting = msg.timestamp
    elif msg.rgv_id == RGV_ID.RGV2:
        _time_of_most_recent_rgv_2_sighting = msg.timestamp
    elif msg.rgv_id == RGV_ID.RGVBOTH:
        _time_of_most_recent_rgv_1_sighting = msg.timestamp
        _time_of_most_recent_rgv_2_sighting = msg.timestamp
    else:
        raise Exception(f"Unrecognized RGV id {msg.rgv_id}")


def _mission_state_callback(msg: MissionState):
    """
    This is the callback used by the mission state subscriber. It updates _mission_state
    to the new value and updates the mission-state-based values
    """
    
    global _current_mission_state, _current_mission_state_start_time, _current_mission_state_time_spent
    
    rospy.logdebug("State machine criteria generator saved a mission state")
    new_mission_state = MissionStates(msg.mission_state)
    if new_mission_state != _current_mission_state:
        # Change to new mission state
        _current_mission_state = new_mission_state
        _current_mission_state_start_time = msg.timestamp


def _battery_callback(msg: BatteryState):
    """
    This is the callback used by the battery subscriber. It updates _low_battery based on how charged
    the battery is.
    """
    
    global _low_battery
    
    rospy.logdebug("State machine criteria generator saved a battery state")
    _low_battery = msg.percentage <= BATTERY_LOW_CHARGE_PCT_CUTOFF
        

def setup():
    """
    Setup publishers and subscribers for generate_state_machine_criteria.py
    """
    
    global _state_machine_criteria_pub, _estimated_rgv_state_sub, _sightings_sub, _mission_state_sub, _battery_sub, _current_mission_state_start_time
    
    _state_machine_criteria_pub = rospy.Publisher(STATE_MACHINE_CRITERIA, StateMachineCriteria, queue_size=1)
    _estimated_rgv_state_sub = rospy.Subscriber(ESTIMATED_RGV_STATES, EstimatedRgvState, _estimated_rgv_state_callback)
    _sightings_sub = rospy.Subscriber(RECENT_RGV_SIGHTINGS, RecentSighting, _sightings_callback)
    _mission_state_sub = rospy.Subscriber(MISSION_STATES, MissionState, _mission_state_callback)
    _battery_sub = rospy.Subscriber(BATTERY, BatteryState, _battery_callback)
    _current_mission_state_start_time = rospy.Time.now()


_state_machine_criteria_pub: rospy.Publisher
_estimated_rgv_state_sub: rospy.Subscriber
_sightings_sub: rospy.Subscriber
_mission_state_sub: rospy.Subscriber
_battery_sub : rospy.Subscriber
