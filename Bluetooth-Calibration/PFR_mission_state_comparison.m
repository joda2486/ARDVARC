clc
clear
close all

[file, location] = uigetfile("*.bag", "Flight Test Bag");
flight_bag = rosbag([location, file]);

[file, location] = uigetfile("*.bag", "Simulation Bag");
sim_bag = rosbag([location, file]);

flight_mission_states = extract_mission_states(flight_bag);
flight_start_index_temp = find(flight_mission_states.MissionState == 1, 1);
flight_start_time = flight_mission_states.Time(flight_start_index_temp) - 30;
flight_start_index = find(flight_mission_states.Time > flight_start_time, 1);
flight_end_index = find(flight_mission_states.MissionState == 6, 1, "last");
flight_mission_states = flight_mission_states(flight_start_index:flight_end_index,:);
flight_mission_states.Time = flight_mission_states.Time - flight_mission_states.Time(1);

bag_mission_states = extract_mission_states(sim_bag);
bag_start_index_temp = find(bag_mission_states.MissionState == 1, 1);
bag_start_time = bag_mission_states.Time(bag_start_index_temp) - 30;
bag_start_index = find(bag_mission_states.Time > bag_start_time, 1);
bag_end_index = find(bag_mission_states.MissionState == 6, 1, "last");
bag_mission_states = bag_mission_states(bag_start_index:bag_end_index,:);
bag_mission_states.Time = bag_mission_states.Time - bag_mission_states.Time(1);

xmax = max(max(flight_mission_states.Time), max(bag_mission_states.Time))/60;

f = figure;
subplot(1,2,1)
hold on
grid on
grid minor
stairs(flight_mission_states.Time./60, flight_mission_states.MissionState, 'r')
xlim([0 xmax])
ylim([-1 7])
yticks(0:6)
yticklabels(["Find RGV 1", "Trail RGV 1", "Localize RGV 1", "Find RGV 2", "Trail RGV 2", "Localize RGV 2", "Joint Localize"])
title("Flight Test Mission States Over Time")

subplot(1,2,2)
hold on
grid on
grid minor
stairs(bag_mission_states.Time./60, bag_mission_states.MissionState, 'r')
xlim([0 xmax])
ylim([-1 7])
yticklabels("")
title("Simulation Mission States Over Time")

a = axes(f, Visible="off");
a.XLabel.Visible = "on";
xlabel(a, "Time Since Mission Start [min]");