% Extracts mission states from a bag and puts them into a table
function mission_states = extract_mission_states(bag)    
    mission_states_select = select(bag, "Topic", "/state_machine/mission_states");
    mission_states = struct2table(cell2mat(readMessages(mission_states_select,'DataFormat','struct')));
    mission_states.MessageType = [];
    mission_states.FINDRGV1 = [];
    mission_states.TRACKRGV1 = [];
    mission_states.LOCALIZERGV1 = [];
    mission_states.FINDRGV2 = [];
    mission_states.TRACKRGV2 = [];
    mission_states.LOCALIZERGV2 = [];
    mission_states.JOINTLOCALIZE = [];
    mission_states.MissionState = mission_states.MissionState_;
    mission_states.MissionState_ = [];
    mission_states.Time = double([mission_states.Timestamp.Sec]') + 1e-9 * double([mission_states.Timestamp.Nsec]') - bag.StartTime;
    mission_states.Timestamp = [];
end