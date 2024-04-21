% Adds colored vertical regions to a graph showing which mission state the
% drone was in at each timestep.
function plot_mission_state_regions(fig, bag, ymin, ymax)
    mission_states = extract_mission_states(bag);
    
    xmin = 0;
    xmax = bag.EndTime - bag.StartTime;
    
    figure(fig)
    hold on
    grid on
    grid minor
    
    changes = [false; mission_states.MissionState(1:end-1) ~= mission_states.MissionState(2:end)];
    change_times = mission_states.Time(changes)';
    xs = [[xmin change_times];[change_times, xmax];[change_times, xmax];[xmin change_times]];
    ys = [(ones(length(change_times)+1,2)*ymin)';(ones(length(change_times)+1,2)*ymax)'];
    
    colors = [[255, 0, 0];[255, 162, 0];[255, 225, 0];[127, 255, 0];[0, 255, 255];[0, 0, 255];[127, 0, 255]]/255;
    min_state = min(mission_states.MissionState);
    max_state = max(mission_states.MissionState);
    colors = colors(min_state+1:max_state+1,:);
    
    fill(xs,ys,[mission_states.MissionState(1), mission_states.MissionState(changes)'], EdgeColor="none", FaceAlpha=0.1);
    colormap(colors)
end