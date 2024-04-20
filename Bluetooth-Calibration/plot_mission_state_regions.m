% Adds colored vertical regions to a graph showing which mission state the
% drone was in at each timestep.
function plot_mission_state_regions(fig, bag)
    mission_states_select = select(bag, "Topic", "/state_machine/mission_states");
    mission_state_msgs = cell2mat(readMessages(mission_states_select,'DataFormat','struct'));
    mission_state_time_msgs = [mission_state_msgs.Timestamp];
    mission_state_times = double([mission_state_time_msgs.Sec]) + 1e-9 * double([mission_state_time_msgs.Nsec]);
    mission_state_times = mission_state_times - mission_state_times(1);
    mission_states = [mission_state_msgs.MissionState_];
    
    xmin = bag.StartTime;
    xmax = bag.EndTime;
    ymin = -100;
    ymax = 100;
    
    figure(fig)
    hold on
    grid on
    grid minor
    xlim([xmin xmax])
    ylim([ymin ymax])
    
    changes = [false, mission_states(1:end-1) ~= mission_states(2:end)];
    change_times = mission_state_times(changes);
    xs = [[xmin change_times];[change_times, xmax];[change_times, xmax];[xmin change_times]];
    ys = [(ones(length(change_times)+1,2)*ymin)';(ones(length(change_times)+1,2)*ymax)'];
    
    colors = [[255, 0, 0];[255, 162, 0];[255, 225, 0];[127, 255, 0];[0, 255, 255];[0, 0, 255];[127, 0, 255]]/255;
    min_state = min(mission_states);
    max_state = max(mission_states);
    colors = colors(min_state+1:max_state+1,:);
    
    fillplot = fill(xs,ys,[mission_states(1), mission_states(changes)], EdgeColor="none", FaceAlpha=0.1);
    colormap(colors)
end