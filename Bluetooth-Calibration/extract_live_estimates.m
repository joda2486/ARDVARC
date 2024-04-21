% Extracts RGV estimates from a bag and puts them into a table
function live_estimates = extract_live_estimates(bag)
    live_estimates_select = select(bag, "Topic", "/estimation/estimated_rgv_states");
    live_estimates = struct2table(cell2mat(readMessages(live_estimates_select,'DataFormat','struct')));
    live_estimates.MessageType = [];
    live_estimates.Time = double([live_estimates.Timestamp.Sec]') + 1e-9 * double([live_estimates.Timestamp.Nsec]') - bag.StartTime;
    live_estimates.Timestamp = [];
    live_estimates.Position = cell2mat(live_estimates.Rgv1PositionLocal')';
    live_estimates.Position(:,:,2) = cell2mat(live_estimates.Rgv1PositionLocal')';
    live_estimates.Moving = live_estimates.Rgv1Moving;
    live_estimates.Moving(:,:,2) = live_estimates.Rgv2Moving;
    live_estimates.Speed = live_estimates.Rgv1Speed;
    live_estimates.Speed(:,:,2) = live_estimates.Rgv2Speed;
    live_estimates.Confidence = live_estimates.Rgv1Confidence;
    live_estimates.Confidence(:,:,2) = live_estimates.Rgv2Confidence;
    live_estimates.Rgv1PositionLocal = [];
    live_estimates.Rgv2PositionLocal = [];
    live_estimates.Rgv1Moving = [];
    live_estimates.Rgv2Moving = [];
    live_estimates.Rgv1Speed = [];
    live_estimates.Rgv2Speed = [];
    live_estimates.Rgv1Confidence = [];
    live_estimates.Rgv2Confidence = [];
end