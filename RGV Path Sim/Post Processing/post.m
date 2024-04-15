clc; close all


%% Initialize
% Read File
bag = rosbag('ardvarc_sim_bag_001_100.bag');

rosbag info 'ardvarc_sim_bag_001_100.bag' % display information



% Initilize Variables
bluebag = select(bag,'Topic','/bluetooth/az_els'); % BluetoothAzimuthElevation
bluestruct = readMessages(bluebag,'DataFormat','Struct');

cambag = select(bag,'Topic','/camera/frames'); % image files 
camstruct = readMessages(cambag,'DataFormat','Struct');

UAS_to_RGV_bag = select(bag,'Topic','/estimation/direction_vectors_uas'); % UasToRgvDirectionVectorUasFrame
UAS_to_RGV_struct = readMessages(UAS_to_RGV_bag,'DataFormat','Struct');

RGV_state_bag = select(bag,'Topic','/estimation/estimated_rgv_states'); % Estimated RGV state
RGV_state_struct = readMessages(RGV_state_bag,'DataFormat','Struct');

mission_states_bag = select(bag,'Topic','/state_machine/mission_states');
mission_states_struct = readMessages(mission_states_bag,'DataFormat','Struct');

posebag = select(bag,'Topic','/mavros/setpoint_position/local'); % poses
posestruct = readMessages(posebag,'DataFormat','Struct');



%{
%% RGV Location: Bluetooth and Frame Transformations
% run transformation function on bluetooth data
% maybe don't need -- for now using UAStoRGV direction vector




%% Camera Files
% read image, identify bounding box
img = rosReadImage(msg);

%% Poses

%}
%% Mission State and Estimated RGV State



m_states = zeros(length(mission_states_struct),2);
for ii = 1:length(mission_states_struct)
    m_states(ii,1) = double(mission_states_struct{ii, 1}.Timestamp.Nsec)*10^-9 + double(mission_states_struct{ii, 1}.Timestamp.Sec);
    m_states(ii,2) = mission_states_struct{ii, 1}.MissionState_;  
end

rgv_states = zeros(length(RGV_state_struct),3);
for ii = 1:length(RGV_state_struct)
    rgv_states(ii,1) = RGV_state_struct{ii, 1}.Timestamp.Sec;
    rgv_states(ii,2) = RGV_state_struct{ii, 1}.Rgv1Moving;
    rgv_states(ii,3) = RGV_state_struct{ii, 1}.Rgv2Moving;
end

%{
%% Plots

% plot UAS path
xPoints = cellfun(@(m) double(m.X),posestruct);
yPoints = cellfun(@(m) double(m.Y),posestruct);
zPoints = cellfun(@(m) double(m.Z),posestruct);
plot(xPoints,yPoints,zPoints)

% plot RGV paths



%% Output



%}
%% Functions

% need function to take a matrix where column 1 is timestamps and column 2
% is data values, and remove repetitive timestamps -- ex. see matrix m_states
% or set timestamp divisions

% need function to take multiple matrices and compare timestamps, align and
% trim timestamps to lowest common denominator, and combine into one large matrix


