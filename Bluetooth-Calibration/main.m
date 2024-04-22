clc
clear

[file, location] = uigetfile("*.bag");
bag = rosbag([location, file]);

if isfile(file+"cv.mat")
    disp("Using existing post CV")
    load(file+"cv.mat", "cv_direction_vectors")
else
    disp("Doing CV in post...")
    cv_direction_vectors = cv_post(bag);
    save(file+"cv.mat", "cv_direction_vectors")
end

disp("Extracting relevant values")
direction_vectors = extract_direction_vectors(bag);
mask = direction_vectors.MeasurementSource == constants.BLUETOOTH_SOURCE;
bluetooth_direction_vectors = direction_vectors(mask,:);
poses = extract_uas_poses(bag);

disp("Calculating and applying optimal sensor delays")
ideal_bluetooth_delay = fminsearch(@(delay) calculate_best_windowed_unbiased_2drms(poses, bluetooth_direction_vectors, delay, constants.BLUETOOTH_SOURCE, 90), 0);
ideal_camera_delay = fminsearch(@(delay) calculate_best_windowed_unbiased_2drms(poses, cv_direction_vectors, delay, constants.CAMERA_SOURCE, 90), 0);
fprintf("Bluetooth Delay: %2.2f\n", ideal_bluetooth_delay)
fprintf("Camera Delay: %2.2f\n", ideal_camera_delay)

disp("Calculating projections for each RGV and sensor")
bluetooth_projections_rgv1 = calculate_projections(poses,bluetooth_direction_vectors,ideal_bluetooth_delay,constants.BLUETOOTH_SOURCE,1);
bluetooth_projections_rgv2 = calculate_projections(poses,bluetooth_direction_vectors,ideal_bluetooth_delay,constants.BLUETOOTH_SOURCE,2);
cv_projections_rgv1 = calculate_projections(poses,cv_direction_vectors,ideal_bluetooth_delay,constants.CAMERA_SOURCE,1);
cv_projections_rgv2 = calculate_projections(poses,cv_direction_vectors,ideal_bluetooth_delay,constants.CAMERA_SOURCE,2);



disp("Calculating camera-only estimates")
estimate_times = 0:constants.ESTIMATE_GAP:(bag.EndTime-bag.StartTime);
rgv1_coarse_estimates = zeros(length(estimate_times),2);
rgv2_coarse_estimates = zeros(length(estimate_times),2);
for i = 1:length(estimate_times)
    estimate_time = estimate_times(i);
    rgv1_coarse_estimates(i,:) = calculate_estimate(bluetooth_projections_rgv1([],:), cv_projections_rgv1, estimate_time, constants.ESTIMATE_WINDOW_SIZE);
    rgv2_coarse_estimates(i,:) = calculate_estimate(bluetooth_projections_rgv2([],:), cv_projections_rgv2, estimate_time, constants.ESTIMATE_WINDOW_SIZE);
end

disp("Calculating multi-sensor estimates")
rgv1_fine_estimates = zeros(length(estimate_times),2);
rgv2_fine_estimates = zeros(length(estimate_times),2);
for i = 1:length(estimate_times)
    estimate_time = estimate_times(i);
    rgv1_fine_estimates(i,:) = calculate_estimate(bluetooth_projections_rgv1, cv_projections_rgv1, estimate_time, constants.ESTIMATE_WINDOW_SIZE);
    rgv2_fine_estimates(i,:) = calculate_estimate(bluetooth_projections_rgv2, cv_projections_rgv2, estimate_time, constants.ESTIMATE_WINDOW_SIZE);
end

disp("Calculating windowed unbiased 2DRMS for both estimate types and RGVs")
coarse_half_index_offset = ceil(30/constants.ESTIMATE_GAP);
fine_half_index_offset = ceil(15/constants.ESTIMATE_GAP);
joint_half_index_offset = ceil(20/constants.ESTIMATE_GAP);
coarse_fine_index_offset = ceil(45/constants.ESTIMATE_GAP);
rgv1_coarse_2drms = calculate_windowed_unbiased_2drms(rgv1_coarse_estimates, estimate_times, 60);
rgv1_coarse_2drms(1:coarse_half_index_offset) = NaN;
rgv1_coarse_2drms(end-coarse_half_index_offset-2*fine_half_index_offset:end) = NaN;
rgv2_coarse_2drms = calculate_windowed_unbiased_2drms(rgv2_coarse_estimates, estimate_times, 60);
rgv2_coarse_2drms(1:coarse_half_index_offset) = NaN;
rgv2_coarse_2drms(end-coarse_half_index_offset-2*fine_half_index_offset:end) = NaN;
rgv1_fine_2drms = calculate_windowed_unbiased_2drms(rgv1_fine_estimates, estimate_times, 30);
rgv2_fine_2drms = calculate_windowed_unbiased_2drms(rgv2_fine_estimates, estimate_times, 30);
rgv1_joint_2drms = calculate_windowed_unbiased_2drms(rgv1_fine_estimates, estimate_times, 20);
rgv2_joint_2drms = calculate_windowed_unbiased_2drms(rgv2_fine_estimates, estimate_times, 20);
rgv1_joint_2drms(1:fine_half_index_offset) = NaN;
rgv2_joint_2drms(1:fine_half_index_offset) = NaN;

disp("Determining best windows of coarse and fine data")
rgv1_2drms_cost = rgv1_coarse_2drms(1:end-coarse_fine_index_offset+1) + rgv1_fine_2drms(coarse_fine_index_offset:end);
rgv2_2drms_cost = rgv2_coarse_2drms(1:end-coarse_fine_index_offset+1) + rgv2_fine_2drms(coarse_fine_index_offset:end);
[~,rgv1_coarse_index] = min(rgv1_2drms_cost);
rgv2_2drms_cost(rgv1_coarse_index-coarse_half_index_offset:rgv1_coarse_index+coarse_half_index_offset+2*fine_half_index_offset) = NaN;
[~,rgv2_coarse_index] = min(rgv2_2drms_cost);
rgv1_fine_index = rgv1_coarse_index + coarse_fine_index_offset;
rgv2_fine_index = rgv2_coarse_index + coarse_fine_index_offset;

fprintf("RGV 1 Coarse Localization Unbiased 2DRMS: %2.2f\n", rgv1_coarse_2drms(rgv1_coarse_index))
fprintf("RGV 2 Coarse Localization Unbiased 2DRMS: %2.2f\n", rgv2_coarse_2drms(rgv2_coarse_index))
fprintf("RGV 1 Fine Localization Unbiased 2DRMS: %2.2f\n", rgv1_fine_2drms(rgv1_fine_index))
fprintf("RGV 2 Fine Localization Unbiased 2DRMS: %2.2f\n", rgv2_fine_2drms(rgv2_fine_index))

disp("Determining best window of joint data")
joint_2drms_cost = rgv1_fine_2drms.^2+rgv2_fine_2drms.^2;
% joint_2drms_cost(1:rgv1_fine_index+fine_half_index_offset+joint_half_index_offset) = NaN;
% joint_2drms_cost(1:rgv2_fine_index+fine_half_index_offset+joint_half_index_offset) = NaN;
% joint_2drms_cost = joint_2drms_cost(1:length(estimate_times));
[~,joint_index] = min(joint_2drms_cost);
fprintf("RGV 1 Joint Localization Unbiased 2DRMS: %2.2f\n", rgv1_fine_2drms(joint_index))
fprintf("RGV 2 Joint Localization Unbiased 2DRMS: %2.2f\n", rgv2_fine_2drms(joint_index))

rgv1_joint_estimates = rgv1_fine_estimates(joint_index-joint_half_index_offset:joint_index+joint_half_index_offset,:);
rgv2_joint_estimates = rgv2_fine_estimates(joint_index-joint_half_index_offset:joint_index+joint_half_index_offset,:);
rgv1_coarse_estimates = rgv1_coarse_estimates(rgv1_coarse_index-coarse_half_index_offset:rgv1_coarse_index+coarse_half_index_offset,:);
rgv2_coarse_estimates = rgv2_coarse_estimates(rgv2_coarse_index-coarse_half_index_offset:rgv2_coarse_index+coarse_half_index_offset,:);
rgv1_fine_estimates = rgv1_fine_estimates(rgv1_fine_index-fine_half_index_offset:rgv1_fine_index+fine_half_index_offset,:);
rgv2_fine_estimates = rgv2_fine_estimates(rgv2_fine_index-fine_half_index_offset:rgv2_fine_index+fine_half_index_offset,:);

%% ERE
disp("Loading the truth RGV estimates")
%%Assign the Truth Values
% [file, location] = uigetfile("*.bag"); %%Use the UI to grab the truth
% vector
truth_joint_1 = [3.3685 -4.8270];
truth_joint_2 = [-1.7722 -4.8270];
truth_coarse_1 = [3.3685 -4.8270];
truth_coarse_2 = [-1.7722 -4.8270];
truth_fine_1 = [3.3685 -4.8270];
truth_fine_2 = [-1.7722 -4.8270];


rgv1_joint_truth = ones(length(rgv1_joint_estimates),2) .* truth_joint_1;
rgv2_joint_truth = ones(length(rgv2_joint_estimates),2) .* truth_joint_2;
rgv1_coarse_truth = ones(length(rgv1_coarse_estimates),2) .* truth_coarse_1;
rgv2_coarse_truth = ones(length(rgv2_coarse_estimates),2) .* truth_coarse_2;
rgv1_fine_truth = ones(length(rgv1_fine_estimates),2) .* truth_fine_1;
rgv2_fine_truth = ones(length(rgv2_fine_estimates),2) .* truth_fine_2;

joint_time = estimate_times(joint_index-joint_half_index_offset:joint_index+joint_half_index_offset);
course_1_time = estimate_times(rgv1_coarse_index-coarse_half_index_offset:rgv1_coarse_index+coarse_half_index_offset);
course_2_time = estimate_times(rgv2_coarse_index-coarse_half_index_offset:rgv2_coarse_index+coarse_half_index_offset);
fine_1_time = estimate_times(rgv1_fine_index-fine_half_index_offset:rgv1_fine_index+fine_half_index_offset);
fine_2_time = estimate_times(rgv2_fine_index-fine_half_index_offset:rgv2_fine_index+fine_half_index_offset);

disp("Calculating the 2DRMS at the estimates")
twodrms_coarse_1 = calculate_best_windowed_2drms_truth(rgv1_coarse_estimates,course_1_time,rgv1_coarse_truth,course_1_time);
twodrms_coarse_2 = calculate_best_windowed_2drms_truth(rgv2_coarse_estimates,course_2_time,rgv2_coarse_truth,course_2_time);
twodrms_fine_1 = calculate_best_windowed_2drms_truth(rgv1_fine_estimates,fine_1_time,rgv1_fine_truth,fine_1_time);
twodrms_fine_2 = calculate_best_windowed_2drms_truth(rgv2_fine_estimates,fine_2_time,rgv2_fine_truth,fine_2_time);
twodrms_joint_1 = calculate_best_windowed_2drms_truth(rgv1_joint_estimates,joint_time,rgv1_joint_truth,joint_time);
twodrms_joint_2 = calculate_best_windowed_2drms_truth(rgv2_joint_estimates,joint_time,rgv2_joint_truth,joint_time);

fprintf("RGV 1 Coarse Localization 2DRMS: %2.2f\n", twodrms_coarse_1)
fprintf("RGV 2 Coarse Localization 2DRMS: %2.2f\n", twodrms_coarse_2)
fprintf("RGV 1 Fine Localization 2DRMS: %2.2f\n", twodrms_fine_1)
fprintf("RGV 2 Fine Localization 2DRMS: %2.2f\n", twodrms_fine_2)
fprintf("RGV 1 Joint Localization 2DRMS: %2.2f\n", twodrms_joint_1)
fprintf("RGV 2 Joint Localization 2DRMS: %2.2f\n", twodrms_joint_2)

%%Plotting 
close all;
% RGV 1 Coarse
figure;hold on;
[P1,P69] = error_ellipse(rgv1_coarse_estimates);
P2 = plot(truth_coarse_1(1),truth_coarse_1(2),'.','Color','b','MarkerSize',10);
P3 = viscircles(truth_coarse_1,twodrms_coarse_1,'Color','b');
P4 = viscircles(truth_coarse_1,2,'Color','g');
axis equal
grid on
grid minor
title("RGV 1 Coarse")
legend([P1,P69,P2,P3,P4],"Estimates","95% Confidence Interval","True Location","Calculated 2DRMS","2 Meter 2DRMS")
xlabel("E [m]")
ylabel("N [m]")
% RGV 2 Coarse
figure; hold on;
[P1,P69] = error_ellipse(rgv2_coarse_estimates);
P2 = plot(truth_coarse_2(1),truth_coarse_2(2),'.','Color','b','MarkerSize',10);
P3 = viscircles(truth_coarse_2,twodrms_coarse_2,'Color','b');
P4 = viscircles(truth_coarse_2,2,'Color','g');
axis equal
grid on
grid minor
title("RGV 2 Coarse")
legend([P1,P69,P2,P3,P4],"Estimates","95% Confidence Interval","True Location","Calculated 2DRMS","2 Meter 2DRMS")
xlabel("E [m]")
ylabel("N [m]")
% RGV 1 Fine
figure; hold on;
[P1,P69] = error_ellipse(rgv1_fine_estimates);
P2 = plot(truth_fine_1(1),truth_fine_1(2),'.','Color','b','MarkerSize',10);
P3 = viscircles(truth_fine_1,twodrms_fine_1,'Color','b');
P4 = viscircles(truth_fine_1,1,'Color','g');
axis equal
grid on
grid minor
title("RGV 1 Fine")
legend([P1,P69,P2,P3,P4],"Estimates","95% Confidence Interval","True Location","Calculated 2DRMS","1 Meter 2DRMS")
xlabel("E [m]")
ylabel("N [m]")
% RGV 2 Fine
figure; hold on;
[P1,P69] = error_ellipse(rgv2_fine_estimates);
P2 = plot(truth_fine_2(1),truth_fine_2(2),'.','Color','b','MarkerSize',10);
P3 = viscircles(truth_fine_2,twodrms_fine_2,'Color','b');
P4 = viscircles(truth_fine_2,1,'Color','g');
axis equal
grid on
grid minor
title("RGV 2 Fine")
legend([P1,P69,P2,P3,P4],"Estimates","95% Confidence Interval","True Location","Calculated 2DRMS","1 Meter 2DRMS")
xlabel("E [m]")
ylabel("N [m]")
% Joint
figure;hold on;
[P1,P69] = error_ellipse(rgv1_joint_estimates);
P2 = plot(truth_joint_1(1),truth_joint_1(2),'.','Color','b','MarkerSize',10);
P3 = viscircles(truth_joint_1,twodrms_joint_1,'Color','b');
P4 = viscircles(truth_joint_1,1,'Color','g');

[P11,P691] = error_ellipse(rgv2_joint_estimates);
P21 = plot(truth_joint_2(1),truth_joint_2(2),'o','Color','b','MarkerSize',10);
P31 = viscircles(truth_joint_2,twodrms_joint_2,'Color','b','LineStyle','--');
P41 = viscircles(truth_joint_2,1,'Color','g','LineStyle','--');

P691.Color = "r";
P691.LineStyle = "--";
axis equal
grid on
grid minor
title("Joint Localization")
legend([P1,P69,P2,P3,P4,P11,P691,P21,P31,P41],"Estimates - RGV1","95% Confidence Interval - RGV1","True Location - RGV1","Calculated 2DRMS - RGV1","1 Meter 2DRMS - RGV1","Estimates - RGV2","95% Confidence Interval - RGV2","True Location - RGV2","Calculated 2DRMS - RGV2","1 Meter 2DRMS - RGV2")
xlabel("E [m]")
ylabel("N [m]")

mission_states = extract_mission_states(bag);
figure;hold on;
axis equal
grid on
grid minor
title("Mission States Over Time")
plot(mission_states.Time, mission_states.MissionState)
axis([0 (mission_states.Time(end)+5) -1 7])
axis square




figure
hold on
grid on
grid minor
plot(estimate_times, rgv1_coarse_2drms, "r", DisplayName="RGV 1 Coarse")
plot(estimate_times, rgv2_coarse_2drms, "b", DisplayName="RGV 2 Coarse")
plot(estimate_times, rgv1_fine_2drms, "r--", DisplayName="RGV 1 Fine")
plot(estimate_times, rgv2_fine_2drms, "b--", DisplayName="RGV 2 Fine")
plot(estimate_times(1:end-coarse_fine_index_offset+1), rgv1_2drms_cost, "r:", DisplayName="RGV 1 Cost")
plot(estimate_times(1:end-coarse_fine_index_offset+1), rgv2_2drms_cost, "b:", DisplayName="RGV 2 Cost")
plot(estimate_times, rgv1_joint_2drms, "k", DisplayName="RGV 1 Joint")
plot(estimate_times, rgv2_joint_2drms, "k--", DisplayName="RGV 2 Joint")
plot(estimate_times, joint_2drms_cost, "k:", DisplayName="Joint Cost")
xline(estimate_times(rgv1_coarse_index-coarse_half_index_offset), "r", DisplayName="")
xline(estimate_times(rgv1_fine_index+fine_half_index_offset), "r", DisplayName="")
xline(estimate_times(rgv2_coarse_index-coarse_half_index_offset), "b", DisplayName="")
xline(estimate_times(rgv2_fine_index+fine_half_index_offset), "b", DisplayName="")
xline(estimate_times(joint_index-joint_half_index_offset), "k", DisplayName="")
xline(estimate_times(joint_index+joint_half_index_offset), "k", DisplayName="")
legend(location="best")
ylim([0 10])

