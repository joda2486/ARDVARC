% Plots projections (birds-eye view) over a slected window, adding the
% specified delay.
%
% Make sure to account for any existing measurement delay already factored
% into the recorded pointing vectors.

clc
clear
close all

[file, location] = uigetfile("*.bag");
bag = rosbag([location, file]);

plot_focus_time = 108; % Window is centered on this time
plot_duration = 90;
bt_delay = -0.3;
cam_delay = 0.5;
rgv_id = 1;

poses = extract_uas_poses(bag);
[bt_times, bt_projections] = calculate_projections(bag, poses, bt_delay, constants.BLUETOOTH_SOURCE, rgv_id);
bt_times = bt_times - bag.StartTime;
bt_projections = bt_projections(bt_times >= plot_focus_time - plot_duration/2 & bt_times <= plot_focus_time + plot_duration/2,:);

[cam_times, cam_projections] = calculate_projections(bag, poses, cam_delay, constants.CAMERA_SOURCE, rgv_id);
cam_times = cam_times - bag.StartTime;
cam_projections = cam_projections(cam_times >= plot_focus_time - plot_duration/2 & cam_times <= plot_focus_time + plot_duration/2,:);

figure
hold on
grid on
grid minor
axis equal
scatter(bt_projections(:,1), bt_projections(:,2), 'k.')
scatter(cam_projections(:,1), cam_projections(:,2), 'r.')