import unittest
from FSW.config.structures import MissionStates
from FSW.config.constants import LOCALIZE_DURATION
import rospy
from rosardvarc.msg import StateMachineCriteria

class TestStateMachineCriteriaGenerator(unittest.TestCase):
    def test_finish_localizing_after_time(self):
        import FSW.functional.generate_state_machine_criteria as gsmc
        
        # Change module global variables so that they should result in
        # a message that says we are done localizing
        now = rospy.Time.from_sec(1000)
        gsmc._current_mission_state = MissionStates.LOCALIZE_RGV_1
        gsmc._current_mission_state_start_time = now - LOCALIZE_DURATION
                
        # Create the message
        msg = gsmc._build_state_machine_criteria_message(now)
        
        # Check that the message says that we are done localizing
        self.assertTrue(msg.rgv_1_localized)
    
    def test_dont_finish_localizing_early(self):
        import FSW.functional.generate_state_machine_criteria as gsmc
        
        # Change module global variables so that they should result in
        # a message that says we are not done localizing
        now = rospy.Time.from_sec(1000)
        gsmc._current_mission_state = MissionStates.LOCALIZE_RGV_1
        gsmc._current_mission_state_start_time = now - LOCALIZE_DURATION * 0.9
                
        # Create the message
        msg = gsmc._build_state_machine_criteria_message(now)
        
        # Check that the message does not say that we are done localizing
        self.assertFalse(msg.rgv_1_localized)
        

class TestDetermineMissionState(unittest.TestCase):
    def test_transition_from_track_to_localize(self):
        import FSW.functional.determine_mission_state as dms
        
        # Set current state to TRACK_RGV_1
        current_state = MissionStates.TRACK_RGV_1
        
        # Create a criteria message that should trigger the transition to
        # LOCALIZE_RGV_1
        criteria = StateMachineCriteria()
        criteria.timestamp = rospy.Time.from_sec(1000)
        criteria.recent_rgv_1_estimate = True
        criteria.recent_rgv_2_estimate = True
        criteria.rgv_1_is_moving = False
        criteria.rgv_2_is_moving = True
        criteria.rgv_1_localized = False
        criteria.rgv_2_localized = False
        criteria.joint_localized = False
        criteria.rgv_1_sighted = True
        criteria.rgv_2_sighted = False
        criteria.minimum_localize_time_reached = False
        criteria.battery_low = False
        
        # Check that the next generated mission state is LOCALIZE_RGV_1
        next_mission_state = dms._determine_next_mission_state(current_state, criteria)
        self.assertEqual(next_mission_state, MissionStates.LOCALIZE_RGV_1)
    
    
    def test_transition_from_localize_1_to_find_2(self):
        import FSW.functional.determine_mission_state as dms
        
        # Set current state to LOCALIZE_RGV_1
        current_state = MissionStates.LOCALIZE_RGV_1
        
        # Create a criteria message that should trigger the transition to
        # FIND_RGV_2
        criteria = StateMachineCriteria()
        criteria.timestamp = rospy.Time.from_sec(1000)
        criteria.recent_rgv_1_estimate = True
        criteria.recent_rgv_2_estimate = False
        criteria.rgv_1_is_moving = False
        criteria.rgv_2_is_moving = False
        criteria.rgv_1_localized = True
        criteria.rgv_2_localized = False
        criteria.joint_localized = False
        criteria.rgv_1_sighted = True
        criteria.rgv_2_sighted = False
        criteria.minimum_localize_time_reached = False
        criteria.battery_low = False
        
        # Check that the next generated mission state is FIND_RGV_2
        next_mission_state = dms._determine_next_mission_state(current_state, criteria)
        self.assertEqual(next_mission_state, MissionStates.FIND_RGV_2)
