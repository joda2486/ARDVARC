clc; close all


%% Initialize
% Read File
%data = 'ardvarc_sim_bag_001_100.bag';
%data = '_2024-04-12-15-34-36_1.bag';
%data = '_2024-04-14-17-46-38_0.bag';

bag = rosbag('_2024-04-12-15-34-36_1.bag');
%rosbag info '_2024-04-12-15-34-36_1.bag' % display information


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

posebag = select(bag,'Topic','mavros/local_position/pose'); % poses
posestruct = readMessages(posebag,'DataFormat','Struct');




%% RGV Location: Bluetooth and Frame Transformations
% run transformation function on bluetooth data
% maybe don't need -- for now using UAStoRGV direction vector




%% Camera Files
% read image, identify bounding box
%img = rosReadImage(msg);

%% Poses


UAS_to_RGV1 = zeros(round(length(UAS_to_RGV_struct)/2,TieBreaker="tozero"),4);
UAS_to_RGV2 = zeros(round(length(UAS_to_RGV_struct)/2,TieBreaker="tozero"),4);

if UAS_to_RGV_struct{1, 1}.RgvId == 1
    n = 1;
    o = 0;
elseif UAS_to_RGV_struct{1, 1}.RgvId == 2
    n = 0;
    o = 1;
end

for i = 1:round(length(UAS_to_RGV_struct)/2,TieBreaker="tozero")
    j = 2*i-n;
    UAS_to_RGV1(i,1) = double(UAS_to_RGV_struct{j, 1}.Timestamp.Sec)+double(UAS_to_RGV_struct{j, 1}.Timestamp.Nsec)*10^-9;
    UAS_to_RGV1(i,2) = double(UAS_to_RGV_struct{j, 1}.Direction(1));
    UAS_to_RGV1(i,3) = double(UAS_to_RGV_struct{j, 1}.Direction(2));
    UAS_to_RGV1(i,4) = double(UAS_to_RGV_struct{j, 1}.Direction(3));
 
    k = 2*i-o;
    UAS_to_RGV2(i,1) = double(UAS_to_RGV_struct{k, 1}.Timestamp.Sec)+double(UAS_to_RGV_struct{k, 1}.Timestamp.Nsec)*10^-9;
    UAS_to_RGV2(i,2) = double(UAS_to_RGV_struct{k, 1}.Direction(1));
    UAS_to_RGV2(i,3) = double(UAS_to_RGV_struct{k, 1}.Direction(2));
    UAS_to_RGV2(i,4) = double(UAS_to_RGV_struct{k, 1}.Direction(3));
  
end

UAS_Pose = zeros(length(posestruct),7);
figure
hold on
grid on
for i = 1:length(posestruct)
    UAS_Pose(i,1) = double(posestruct{i, 1}.Header.Stamp.Sec)+double(posestruct{i, 1}.Header.Stamp.Nsec)*10^-9;
    UAS_Pose(i,2) = double(posestruct{i, 1}.Pose.Position.X);
    UAS_Pose(i,3) = double(posestruct{i, 1}.Pose.Position.Y);
    UAS_Pose(i,4) = double(posestruct{i, 1}.Pose.Position.Z);
    UAS_Pose(i,5) = double(posestruct{i, 1}.Pose.Orientation.X);
    UAS_Pose(i,6) = double(posestruct{i, 1}.Pose.Orientation.Y);
    UAS_Pose(i,7) = double(posestruct{i, 1}.Pose.Orientation.Z);

    plot3(UAS_Pose(i,2),UAS_Pose(i,3),UAS_Pose(i,4),Marker='o',Color='r')

end
hold off
%% Mission State and Estimated RGV State



m_states = zeros(length(mission_states_struct),2);
for ii = 1:length(mission_states_struct)
    m_states(ii,1) = double(mission_states_struct{ii, 1}.Timestamp.Nsec)*10^-9 + double(mission_states_struct{ii, 1}.Timestamp.Sec);
    m_states(ii,2) = double(mission_states_struct{ii, 1}.MissionState_);  
end

rgv_states = zeros(length(RGV_state_struct),3);
for ii = 1:length(RGV_state_struct)
    rgv_states(ii,1) = double(RGV_state_struct{ii, 1}.Timestamp.Sec) + double(RGV_state_struct{ii, 1}.Timestamp.Nsec)*10^-9;
    rgv_states(ii,2) = double(RGV_state_struct{ii, 1}.Rgv1Moving);
    rgv_states(ii,3) = double(RGV_state_struct{ii, 1}.Rgv2Moving);
end

%{
%% Plots

% plot UAS path
xPoints = cellfun(@(m) double(m.X),posestruct);
yPoints = cellfun(@(m) double(m.Y),posestruct);
zPoints = cellfun(@(m) double(m.Z),posestruct);
plot(xPoints,yPoints,zPoints)

% plot RGV paths

%}

%% Output

% Alignent Variable
A_var = UAS_to_RGV1;

% Match Mission State to alignment variable data
m_states_aligned = zeros(size(A_var,1),size(m_states,2));

for ii = 1:length(A_var)
[t1,m_idx] = min(abs(m_states(:,1)-A_var(ii,1)));
m_states_aligned(ii,:) = [A_var(ii,1),m_states(m_idx,2)];
end

% Match Poses to alignment variable data
poses_aligned = zeros(size(A_var,1),size(UAS_Pose,2));

for ii = 1:length(A_var)
[t2,p_idx] = min(abs(UAS_Pose(:,1)-A_var(ii,1)));
poses_aligned(ii,:) = [A_var(ii,1),UAS_Pose(p_idx,2:end)];
end


out = [A_var(:,1),A_var(:,2:end),UAS_to_RGV2(:,2:end),poses_aligned(:,2:end),m_states_aligned(:,2:end)]; %need to add pose
out_table = array2table(out,'VariableNames',{'Timestamp','RGV1 N','RGV1 E','RGV1 D',...
    'RGV2 N','RGV2 E','RGV2 D','UAS Pos. X', 'UAS Pos. Y', 'UAS Pos. Z',...
    'UAS Orient X', 'UAS Orient Y', 'UAS Orient Z', 'Mission State'});



%% Functions

% need function to take a matrix where column 1 is timestamps and column 2
% is data values, and remove repetitive timestamps -- ex. see matrix m_states
% or set timestamp divisions

% need function to take multiple matrices and compare timestamps, align and
% trim timestamps to lowest common denominator, and combine into one large matrix


