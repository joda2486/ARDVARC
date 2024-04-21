% Plots projections (birds-eye view) over a slected window, adding the
% specified delay.
%
% Make sure to account for any existing measurement delay already factored
% into the recorded pointing vectors.

clc
clear

[file, location] = uigetfile("*.bag");
bag = rosbag([location, file]);

plot_focus_time = 108; % Window is centered on this time
plot_duration = 90;
bt_delay = -0.3;
cam_delay = 0.5;
rgv_id = 1;

poses = extract_uas_poses(bag);
direction_vectors = extract_direction_vectors(bag);

bt_projections = calculate_projections(poses, direction_vectors, bt_delay, constants.BLUETOOTH_SOURCE, rgv_id);
bt_projections = bt_projections(bt_projections.Time >= plot_focus_time - plot_duration/2 & bt_projections.Time <= plot_focus_time + plot_duration/2,:);

cam_projections = calculate_projections(poses, direction_vectors, cam_delay, constants.CAMERA_SOURCE, rgv_id);
cam_projections = cam_projections(cam_projections.Time >= plot_focus_time - plot_duration/2 & cam_projections.Time <= plot_focus_time + plot_duration/2,:);

poses = poses(poses.Time >= plot_focus_time - plot_duration/2 & poses.Time <= plot_focus_time + plot_duration/2,:);

figure
hold on
grid on
grid minor
axis equal
scatter(bt_projections.Position(:,1), bt_projections.Position(:,2), 'k.')
scatter(cam_projections.Position(:,1), cam_projections.Position(:,2), 'r.')
plot(poses.Position(:,1), poses.Position(:,2), '-g.')
