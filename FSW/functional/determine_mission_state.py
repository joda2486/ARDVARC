import rospy
from ..config.structures import MissionStates
from ..config.topic_names import MISSION_STATES, STATE_MACHINE_CRITERIA
from rosardvarc.msg import MissionState, StateMachineCriteria


_current_state: MissionStates = MissionStates.FIND_RGV_1


def _state_machine_criteria_callback(msg: StateMachineCriteria):
    """
    This is the callback used by the state machine criteria subscriber. It uses the new criteria along with
    the known current state in order to determine the next state, which it then publishes.
    """
    
    global _current_state
    # Choose the next mission state based on _current_state and msg
    _current_state = _determine_next_mission_state(_current_state, msg)
    
    # Publish new mission state
    mission_state_msg = MissionState()
    mission_state_msg.timestamp = msg.timestamp
    mission_state_msg.mission_state = _current_state.value
    _mission_state_pub.publish(mission_state_msg)
    rospy.logdebug("Mission state published")


def _determine_next_mission_state(current_state: MissionStates, criteria: StateMachineCriteria) -> MissionStates:
    """
    Determines what the next mission state should be based on the current mission state and a set of
    state machine criteria. This is meant to identically implement the state machine design in the
    Software Function Tracker spreadsheet.
    """
    
    if current_state is MissionStates.FIND_RGV_1:
        if criteria.recent_rgv_1_estimate:
            # Change to TRACK_RGV_1
            return MissionStates.TRACK_RGV_1
        else:
            # Stay in FIND_RGV_1
            return MissionStates.FIND_RGV_1
    elif current_state is MissionStates.TRACK_RGV_1:
        if not criteria.recent_rgv_1_estimate:
            # Change to FIND_RGV_1
            return MissionStates.FIND_RGV_1
        elif criteria.rgv_1_is_moving or not criteria.rgv_1_sighted:
            # Stay in TRACK_RGV_1
            return MissionStates.TRACK_RGV_1
        else:
            # Change to LOCALIZE_RGV_1
            return MissionStates.LOCALIZE_RGV_1
    elif current_state is MissionStates.LOCALIZE_RGV_1:
        if not criteria.recent_rgv_1_estimate:
            # Change to FIND_RGV_1
            return MissionStates.FIND_RGV_1
        if criteria.rgv_1_localized:
            # Change to RGV 2
            if criteria.recent_rgv_2_estimate:
                # Change to TRACK_RGV_2
                return MissionStates.TRACK_RGV_2
            else:
                # Change to FIND_RGV_2
                return MissionStates.FIND_RGV_2
        else:
            # Stay on RGV 1
            if (criteria.rgv_1_is_moving and not criteria.minimum_localize_time_reached) or not criteria.rgv_1_sighted:
                # Change to TRACK_RGV_1
                return MissionStates.TRACK_RGV_1
            else:
                # Stay in LOCALIZE_RGV_1
                return MissionStates.LOCALIZE_RGV_1
    elif current_state is MissionStates.FIND_RGV_2:
        if criteria.recent_rgv_2_estimate:
            # Change to TRACK_RGV_2
            return MissionStates.TRACK_RGV_2
        else:
            # Stay in FIND_RGV_2
            return MissionStates.FIND_RGV_2
    elif current_state is MissionStates.TRACK_RGV_2:
        if not criteria.recent_rgv_2_estimate:
            # Change to FIND_RGV_2
            return MissionStates.FIND_RGV_2
        elif criteria.rgv_2_is_moving or not criteria.rgv_2_sighted:
            # Stay in TRACK_RGV_2
            return MissionStates.TRACK_RGV_2
        else:
            # Change to LOCALIZE_RGV_2
            return MissionStates.LOCALIZE_RGV_2
    elif current_state is MissionStates.LOCALIZE_RGV_2:
        if not criteria.recent_rgv_2_estimate:
            # Change to FIND_RGV_2
            return MissionStates.FIND_RGV_2
        if criteria.rgv_2_localized:
            # Change to JOINT_LOCALIZE
            return MissionStates.JOINT_LOCALIZE
        else:
            # Stay on RGV 2
            if (criteria.rgv_2_is_moving and not criteria.minimum_localize_time_reached) or not criteria.rgv_2_sighted:
                # Change to TRACK_RGV_2
                return MissionStates.TRACK_RGV_2
            else:
                # Stay in LOCALIZE_RGV_2
                return MissionStates.LOCALIZE_RGV_2
    elif current_state is MissionStates.JOINT_LOCALIZE:
        if criteria.joint_localized:
            # Restart mission state cycle
            return MissionStates.FIND_RGV_1
        else:
            # Stay in JOINT_LOCALIZE
            return MissionStates.JOINT_LOCALIZE
    
    raise Exception("determine_next_mission_state did not return")


def setup():
    """
    Setup publishers and subscribers for determine_mission_state.py
    """
    
    global _mission_state_pub, _state_machine_criteria_sub
    _mission_state_pub = rospy.Publisher(MISSION_STATES, MissionState, queue_size=1)
    _state_machine_criteria_sub = rospy.Subscriber(STATE_MACHINE_CRITERIA, StateMachineCriteria, _state_machine_criteria_callback)
    

_mission_state_pub: rospy.Publisher
_state_machine_criteria_sub: rospy.Subscriber
