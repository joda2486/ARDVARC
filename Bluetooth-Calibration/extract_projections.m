% Extracts RGV estimates from a bag and puts them into a table
function projections = extract_projections(bag)
    projections_select = select(bag, "Topic", "/estimation/rgv_local_projections");
    projections = struct2table(cell2mat(readMessages(projections_select,'DataFormat','struct')));
    projections.MessageType = [];
    projections.Time = double([projections.Timestamp.Sec]') + 1e-9 * double([projections.Timestamp.Nsec]') - bag.StartTime;
    projections.Timestamp = [];
    projections.Position = cell2mat(projections.RgvPositionLocal')';
    projections.RgvPositionLocal = [];
end