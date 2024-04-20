% Plots windowed unbiased 2DRMS for a particular RGV and measurement source
% for a range of different measurement delay values. Good delay values
% should give lower unbiased 2DRMS values.
% 
% Make sure to account for any existing measurement delay already factored
% into the recorded pointing vectors.

clc
clear
close all

[file, location] = uigetfile("*.bag");
bag = rosbag([location, file]);

measurement_source = constants.BLUETOOTH_SOURCE;
rgv_id = 1;
window_size = 90;

delays = linspace(-1,1,21);
N = length(delays);

figure
hold on
colors = ["#0072BD", "#D95319", "#EDB120", "#7E2F8E", "#77AC30", "#4DBEEE", "#A2142F"];
linestyles = ["-", "--", "-.", ":"];

disp("Trying different delays...")
for i = 1:length(delays)
    disp(i + "/" + length(delays))
    delay = delays(i);

    [times, post_projections] = calculate_projections(bag, delay, measurement_source, rgv_id);
    
    color = colors(mod(i-1,length(colors))+1);
    linestyle = linestyles(floor((i-1)/length(colors))+1);
    plot(times - times(1), 2*sqrt(sum(movstd(post_projections, window_size, "SamplePoints", times).^2, 2)), DisplayName=string(delay), Color=color, LineStyle=linestyle)
end

legend(Location="best")
grid on
grid minor
xlabel("Time Since Mission Start [s]")
ylabel("2DRMS [m]")
title(sprintf("2DRMS of Projections over %2.2fs Window", window_size))