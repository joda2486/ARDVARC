% Extracts UAS poses from a bag and puts them into a nicer struct
function poses = extract_uas_poses(bag)
    poses_select = select(bag, "Topic", "/mavros/local_position/pose");
    poses_msgs = cell2mat(readMessages(poses_select,'DataFormat','struct'));
    pose_header_msgs = [poses_msgs.Header];
    pose_time_msgs = [pose_header_msgs.Stamp];
    poses.times = double([pose_time_msgs.Sec]) + 1e-9 * double([pose_time_msgs.Nsec]);
    pose_pose_msgs = [poses_msgs.Pose];
    pose_position_msgs = [pose_pose_msgs.Position];
    poses.positions = [pose_position_msgs.X; pose_position_msgs.Y; pose_position_msgs.Z];
    pose_quaternion_msgs = [pose_pose_msgs.Orientation];
    poses.quaternions = [pose_quaternion_msgs.W; pose_quaternion_msgs.X; pose_quaternion_msgs.Y; pose_quaternion_msgs.Z]';
end