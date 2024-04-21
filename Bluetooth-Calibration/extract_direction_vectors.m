% Extracts direction vectors from a bag and puts them into a table
function direction_vectors = extract_direction_vectors(bag)
    direction_vectors_select = select(bag, "Topic", "/estimation/direction_vectors_uas");
    direction_vectors = struct2table(cell2mat(readMessages(direction_vectors_select,'DataFormat','struct')));
    direction_vectors.MessageType = [];
    direction_vectors.Time = double([direction_vectors.Timestamp.Sec]') + 1e-9 * double([direction_vectors.Timestamp.Nsec]') - bag.StartTime;
    direction_vectors.Timestamp = [];
    direction_vectors.Direction = cell2mat(direction_vectors.Direction')';
end