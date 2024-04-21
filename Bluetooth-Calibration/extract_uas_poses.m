% Extracts UAS poses from a bag and puts them into a table
function poses = extract_uas_poses(bag)
    poses_select = select(bag, "Topic", "/mavros/local_position/pose");
    poses = struct2table(cell2mat(readMessages(poses_select,'DataFormat','struct')));
    poses.MessageType = [];
    headers = [poses.Header];
    stamps = [headers.Stamp];
    poses.Time = double([stamps.Sec]') + 1e-9 * double([stamps.Nsec]') - bag.StartTime;
    poses.Header = [];
    p = poses.Pose;
    positions = [p.Position];
    poses.Position = [positions.X; positions.Y; positions.Z]';
    orientation = [p.Orientation];
    poses.Orientation = [orientation.W; orientation.X; orientation.Y; orientation.Z]';
    poses.Pose = [];
end