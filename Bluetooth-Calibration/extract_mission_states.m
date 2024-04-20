function mission_states = extract_mission_states(bag)    
    mission_states_select = select(bag, "Topic", "/state_machine/mission_states");
    mission_state_msgs = cell2mat(readMessages(mission_states_select,'DataFormat','struct'));
    mission_state_time_msgs = [mission_state_msgs.Timestamp];
    mission_state_times = double([mission_state_time_msgs.Sec]) + 1e-9 * double([mission_state_time_msgs.Nsec]);
    mission_states.times = mission_state_times;
    mission_states.mission_states = [mission_state_msgs.MissionState_];
end