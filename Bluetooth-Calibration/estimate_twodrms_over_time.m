clc
clear
close all

[file, location] = uigetfile("*.bag");
bag = rosbag([location, file]);

window_size = 90;
rgv_id = 1;

delays = linspace(-1,1,21);
N = length(delays);

poses = extract_uas_poses(bag);
estimates = extract_live_estimates(bag);
twodrms = calculate_windowed_unbiased_2drms(estimates.Position(:,:,rgv_id), estimates.Time, window_size);

plot(estimates.Time, twodrms)

grid on
grid minor
xlabel("Time Since Mission Start [s]")
ylabel("2DRMS [m]")
title(sprintf("2DRMS of Projections over %2.2fs Window", window_size))